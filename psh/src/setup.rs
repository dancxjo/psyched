use anyhow::{bail, Context, Result};
use duct::cmd;
use regex::Regex;
use serde::Deserialize;
use std::collections::HashMap;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use which::which;

use crate::module_runner::{bring_module_up, setup_module};
use crate::service_runner::{bring_service_up, setup_service};

#[derive(Deserialize)]
struct HostConfig {
    host: Host,
    provision: Option<Provision>,
    #[serde(default)]
    features: Features,
    #[serde(default)]
    modules: Vec<ModuleDirective>,
    #[serde(default)]
    services: Vec<ServiceDirective>,
}

#[derive(Deserialize)]
struct Host {
    name: String,
}

#[derive(Deserialize)]
struct Provision {
    scripts: Option<Vec<String>>,
}

#[derive(Debug, Default, Deserialize)]
struct Features {
    #[serde(default)]
    docker: bool,
    #[serde(default)]
    ros2: bool,
    #[serde(default)]
    cuda: bool,
}

#[derive(Deserialize)]
struct ModuleDirective {
    name: String,
    #[serde(default = "default_true")]
    setup: bool,
    #[serde(default)]
    launch: bool,
    #[serde(default)]
    env: HashMap<String, String>,
}

#[derive(Deserialize)]
struct ServiceDirective {
    name: String,
    #[serde(default = "default_true")]
    setup: bool,
    #[serde(default)]
    up: bool,
    #[serde(default)]
    env: HashMap<String, String>,
}

fn default_true() -> bool {
    true
}

struct EnvOverride {
    saved: Vec<(String, Option<String>)>,
}

impl EnvOverride {
    fn apply(overrides: &HashMap<String, String>) -> Self {
        let mut saved = Vec::new();

        for (key, value) in overrides {
            let key_owned = key.to_string();
            saved.push((key_owned.clone(), env::var(&key_owned).ok()));
            // SAFETY: Host provisioning runs in a single-threaded context and
            // orchestrates child processes sequentially, so temporarily
            // mutating the process environment is well-defined for the
            // duration of this guard.
            unsafe {
                std::env::set_var(&key_owned, value);
            }
        }

        Self { saved }
    }
}

impl Drop for EnvOverride {
    fn drop(&mut self) {
        while let Some((key, prior)) = self.saved.pop() {
            match prior {
                Some(value) => unsafe {
                    std::env::set_var(&key, value);
                },
                None => unsafe {
                    std::env::remove_var(&key);
                },
            }
        }
    }
}

pub fn run_setup() -> Result<()> {
    let hostname = hostname::get()
        .context("failed to detect hostname")?
        .to_string_lossy()
        .into_owned();
    println!("Detected hostname: {hostname}");

    let config_path = locate_host_config(&hostname)
        .with_context(|| format!("no host config found for {hostname}"))?;

    println!("Loading host config: {}", config_path.display());
    let toml = fs::read_to_string(&config_path)
        .with_context(|| format!("failed to read {}", config_path.display()))?;

    let cfg: HostConfig = toml::from_str(&toml)
        .with_context(|| format!("failed to parse TOML {}", config_path.display()))?;

    let HostConfig {
        host,
        provision,
        features,
        modules,
        services,
    } = cfg;

    let host_name = host.name;

    let scripts = provision.and_then(|p| p.scripts).unwrap_or_default();
    let scripts = plan_provision_scripts(scripts, &features);

    if scripts.is_empty() {
        println!("No provisioning scripts listed. Nothing to run.");
        let module_failures = process_modules(&host_name, modules)?;
        let service_failures = process_services(&host_name, services)?;

        let mut failures = Vec::new();
        failures.extend(module_failures);
        failures.extend(service_failures);

        if failures.is_empty() {
            println!("PSH setup complete for host '{}'.", host_name);
        } else {
            println!("PSH setup finished with warnings for host '{}':", host_name);
            for failure in failures {
                eprintln!("  - {failure}");
            }
        }
        return Ok(());
    }

    let mut failures = Vec::new();

    for script in scripts {
        println!("Running {script} …");
        let resolved = resolve_script_path(&script, &config_path);

        if !resolved.exists() {
            eprintln!("Script not found: {}", resolved.display());
            failures.push(format!("{script} (missing)"));
            continue;
        }

        match cmd!("bash", &resolved).stderr_to_stdout().run() {
            Ok(_) => println!("{script} complete."),
            Err(err) => {
                eprintln!("Error running {script}: {err}");
                failures.push(format!("{script} (error: {err})"));
            }
        }
    }

    let module_failures = process_modules(&host_name, modules)?;
    let service_failures = process_services(&host_name, services)?;
    failures.extend(module_failures);
    failures.extend(service_failures);

    if failures.is_empty() {
        println!("PSH setup complete for host '{}'", host_name);
    } else {
        println!("PSH setup finished with warnings for host '{}':", host_name);
        for failure in failures {
            eprintln!("  - {failure}");
        }
    }

    Ok(())
}

fn locate_host_config(hostname: &str) -> Option<PathBuf> {
    let mut roots = Vec::new();

    if let Ok(cwd) = env::current_dir() {
        roots.push(cwd);
    }

    if let Ok(exe) = env::current_exe() {
        if let Some(dir) = exe.parent() {
            roots.push(dir.to_path_buf());
            if let Some(parent) = dir.parent() {
                roots.push(parent.to_path_buf());
            }
        }
    }

    roots
        .into_iter()
        .map(|root| root.join("hosts").join(format!("{hostname}.toml")))
        .find(|candidate| candidate.exists())
}

fn resolve_script_path(script: &str, config_path: &Path) -> PathBuf {
    if let Ok(found) = which(script) {
        return found;
    }

    let requested = PathBuf::from(script);
    if requested.is_absolute() {
        return requested;
    }

    let mut candidates = Vec::new();

    if let Some(config_dir) = config_path.parent() {
        candidates.push(config_dir.join(&requested));
        if let Some(repo_root) = config_dir.parent() {
            candidates.push(repo_root.join(&requested));
        }
    }

    if let Ok(cwd) = env::current_dir() {
        candidates.push(cwd.join(&requested));
    }

    for candidate in candidates {
        if candidate.exists() {
            return candidate;
        }
    }

    requested
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ScriptKind {
    Docker,
    GenerateBindings,
    Ros2,
}

fn classify_script(script: &str) -> Option<ScriptKind> {
    if script.ends_with("install_docker.sh") {
        return Some(ScriptKind::Docker);
    }
    if script.ends_with("generate_ros_rust_bindings.sh") {
        return Some(ScriptKind::GenerateBindings);
    }
    if script.ends_with("install_ros2.sh") {
        return Some(ScriptKind::Ros2);
    }
    None
}

/// Merge explicit provisioning scripts with feature-driven defaults while ensuring
/// Docker, ROS 2, and Rust binding generation run in a safe order.
fn plan_provision_scripts(mut scripts: Vec<String>, features: &Features) -> Vec<String> {
    if scripts.is_empty() && !features.docker && !features.ros2 {
        return scripts;
    }

    let mut planned = Vec::new();
    let mut first_feature_index: Option<usize> = None;

    let mut docker_script: Option<String> = None;
    let mut ros2_script: Option<String> = None;
    let mut bindings_script: Option<String> = None;

    let mut docker_present = false;
    let mut ros2_present = false;
    let mut bindings_present = false;

    for script in scripts.drain(..) {
        match classify_script(&script) {
            Some(ScriptKind::Docker) => {
                docker_present = true;
                if docker_script.is_none() {
                    docker_script = Some(script);
                }
                if first_feature_index.is_none() {
                    first_feature_index = Some(planned.len());
                }
            }
            Some(ScriptKind::GenerateBindings) => {
                bindings_present = true;
                if bindings_script.is_none() {
                    bindings_script = Some(script);
                }
                if first_feature_index.is_none() {
                    first_feature_index = Some(planned.len());
                }
            }
            Some(ScriptKind::Ros2) => {
                ros2_present = true;
                if ros2_script.is_none() {
                    ros2_script = Some(script);
                }
                if first_feature_index.is_none() {
                    first_feature_index = Some(planned.len());
                }
            }
            None => planned.push(script),
        }
    }

    let docker_required = docker_present || features.docker;
    let ros2_required = ros2_present || features.ros2;
    let bindings_required = bindings_present || (docker_required && ros2_required);

    if !docker_required && !ros2_required && !bindings_required {
        return planned;
    }

    let mut feature_block = Vec::new();
    if docker_required {
        feature_block
            .push(docker_script.unwrap_or_else(|| "tools/provision/install_docker.sh".to_string()));
    }
    if bindings_required {
        feature_block.push(
            bindings_script
                .unwrap_or_else(|| "tools/provision/generate_ros_rust_bindings.sh".to_string()),
        );
    }
    if ros2_required {
        feature_block
            .push(ros2_script.unwrap_or_else(|| "tools/provision/install_ros2.sh".to_string()));
    }

    let insert_index = first_feature_index.unwrap_or(planned.len());
    planned.splice(insert_index..insert_index, feature_block);
    planned
}

pub(crate) fn locate_repo_root() -> Result<PathBuf> {
    let mut candidates = Vec::new();

    if let Ok(cwd) = env::current_dir() {
        candidates.push(cwd);
    }

    if let Ok(exe) = env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.push(dir.to_path_buf());
            if let Some(parent) = dir.parent() {
                candidates.push(parent.to_path_buf());
            }
        }
    }

    for root in candidates {
        if root.join("modules").is_dir() && root.join("src").is_dir() {
            return Ok(root);
        }
    }

    bail!("could not determine psyched repository root; run 'psh env' from inside the repository");
}

pub fn setup_env() -> Result<()> {
    let ros_distro = env::var("ROS_DISTRO").unwrap_or_else(|_| "kilted".to_string());
    let workspace_root = locate_repo_root().context("failed to locate repository root")?;

    let install_setup = workspace_root.join("install").join("setup.bash");
    let install_local_setup = workspace_root.join("install").join("local_setup.bash");

    let home_dir = directories::BaseDirs::new()
        .context("failed to determine home directory")?
        .home_dir()
        .to_path_buf();
    let bashrc_path = home_dir.join(".bashrc");

    let existing = if bashrc_path.exists() {
        fs::read_to_string(&bashrc_path).context("failed to read ~/.bashrc")?
    } else {
        String::new()
    };

    let cleanup_re =
        Regex::new(r"(?ms)^# Added by psh env\npsyched\(\) \{\n.*?\n\}\n(?:\n)?psyched.*?(?:\n|$)")
            .context("failed to compile cleanup regex")?;
    let cleaned = cleanup_re.replace_all(&existing, "").to_string();

    let mut updated = cleaned.trim_end().to_string();
    if !updated.is_empty() {
        updated.push_str("\n\n");
    }

    let workspace_str = workspace_root.to_string_lossy().replace('"', "\\\"");
    let function_block = format!(
        "# Added by psh env\npsyched() {{\n    local workspace_root=\"{workspace}\"\n    source /opt/ros/{ros}/setup.bash\n    if [ -f \"$workspace_root/install/setup.bash\" ]; then\n        source \"$workspace_root/install/setup.bash\"\n    elif [ -f \"$workspace_root/install/local_setup.bash\" ]; then\n        source \"$workspace_root/install/local_setup.bash\"\n    fi\n}}\n\npsyched  # Auto-activate ROS 2 workspace\n",
        workspace = workspace_str,
        ros = ros_distro
    );

    updated.push_str(&function_block);

    if !updated.ends_with('\n') {
        updated.push('\n');
    }

    fs::write(&bashrc_path, updated).context("failed to update ~/.bashrc")?;

    println!(
        "✓ Updated '{}' to source ROS and the local workspace",
        bashrc_path.display()
    );
    println!("The 'psyched' function now sources:");
    println!("  - /opt/ros/{}/setup.bash", ros_distro);

    if install_setup.exists() {
        println!("  - {}", install_setup.display());
    } else if install_local_setup.exists() {
        println!("  - {}", install_local_setup.display());
    } else {
        println!(
            "  - {} (not found yet; run 'psh mod setup <ros-module>' to build the workspace)",
            workspace_root.join("install").join("setup.bash").display()
        );
    }

    println!(
        "\nTo use it in this shell, run: source {}",
        bashrc_path.display()
    );
    println!("Or simply run: psyched\n");
    println!(
        "Re-run with ROS_DISTRO set if you need a different base (e.g. ROS_DISTRO=humble psh env)."
    );

    Ok(())
}

fn process_modules(host_name: &str, modules: Vec<ModuleDirective>) -> Result<Vec<String>> {
    if modules.is_empty() {
        println!("No modules configured for host '{host_name}'. Skipping module setup.");
        return Ok(Vec::new());
    }

    println!(
        "Processing {} module directive(s) for host '{host_name}'.",
        modules.len()
    );

    let mut failures = Vec::new();

    for ModuleDirective {
        name,
        setup,
        launch,
        env,
    } in modules
    {
        let mut setup_ok = true;
        let env_guard = EnvOverride::apply(&env);

        if setup {
            println!("[{name}] Running module setup …");
            if let Err(err) = setup_module(&name) {
                eprintln!("[{name}] Setup failed: {err}");
                failures.push(format!("{name} (setup: {err})"));
                setup_ok = false;
            } else {
                println!("[{name}] Setup complete.");
            }
        }

        if launch {
            if setup_ok || !setup {
                println!("[{name}] Launching module …");
                if let Err(err) = bring_module_up(&name) {
                    eprintln!("[{name}] Launch failed: {err}");
                    failures.push(format!("{name} (launch: {err})"));
                } else {
                    println!("[{name}] Launch initiated.");
                }
            } else {
                eprintln!("[{name}] Skipping launch because setup failed for this module.");
            }
        }

        drop(env_guard);
    }

    Ok(failures)
}

fn process_services(host_name: &str, services: Vec<ServiceDirective>) -> Result<Vec<String>> {
    if services.is_empty() {
        println!("No services configured for host '{host_name}'. Skipping service setup.");
        return Ok(Vec::new());
    }

    println!(
        "Processing {} service directive(s) for host '{host_name}'.",
        services.len()
    );

    let mut failures = Vec::new();

    for ServiceDirective {
        name,
        setup,
        up,
        env,
    } in services
    {
        let env_guard = EnvOverride::apply(&env);

        let mut setup_ok = true;
        if setup {
            println!("[{name}] Running service setup …");
            if let Err(err) = setup_service(&name) {
                eprintln!("[{name}] Setup failed: {err}");
                failures.push(format!("{name} (setup: {err})"));
                setup_ok = false;
            } else {
                println!("[{name}] Setup complete.");
            }
        }

        if up {
            if setup_ok || !setup {
                println!("[{name}] Starting service …");
                if let Err(err) = bring_service_up(&name) {
                    eprintln!("[{name}] Start failed: {err}");
                    failures.push(format!("{name} (up: {err})"));
                } else {
                    println!("[{name}] Service running.");
                }
            } else {
                eprintln!("[{name}] Skipping start because setup failed.");
            }
        }

        drop(env_guard);
    }

    Ok(failures)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn features(docker: bool, ros2: bool) -> Features {
        Features {
            docker,
            ros2,
            ..Features::default()
        }
    }

    #[test]
    fn retains_unrelated_scripts_when_no_features() {
        let scripts = vec!["tools/provision/install_deno.sh".to_string()];
        let planned = plan_provision_scripts(scripts.clone(), &Features::default());
        assert_eq!(planned, scripts);
    }

    #[test]
    fn injects_defaults_for_docker_and_ros2_features() {
        let planned = plan_provision_scripts(Vec::new(), &features(true, true));
        assert_eq!(
            planned,
            vec![
                "tools/provision/install_docker.sh".to_string(),
                "tools/provision/generate_ros_rust_bindings.sh".to_string(),
                "tools/provision/install_ros2.sh".to_string(),
            ],
        );
    }

    #[test]
    fn preserves_existing_paths_and_order() {
        let scripts = vec![
            "tools/bootstrap/install_ros2.sh".to_string(),
            "tools/bootstrap/install_deno.sh".to_string(),
        ];
        let planned = plan_provision_scripts(scripts, &features(true, true));
        assert_eq!(
            planned,
            vec![
                "tools/provision/install_docker.sh".to_string(),
                "tools/provision/generate_ros_rust_bindings.sh".to_string(),
                "tools/bootstrap/install_ros2.sh".to_string(),
                "tools/bootstrap/install_deno.sh".to_string(),
            ],
        );
    }

    #[test]
    fn respects_manual_docker_script_path() {
        let scripts = vec![
            "tools/bootstrap/install_docker.sh".to_string(),
            "tools/provision/install_ros2.sh".to_string(),
        ];
        let planned = plan_provision_scripts(scripts, &Features::default());
        assert_eq!(
            planned,
            vec![
                "tools/bootstrap/install_docker.sh".to_string(),
                "tools/provision/generate_ros_rust_bindings.sh".to_string(),
                "tools/provision/install_ros2.sh".to_string(),
            ],
        );
    }
}
