use anyhow::{Context, Result, bail};
use duct::cmd;
use regex::Regex;
use serde::Deserialize;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use which::which;

#[derive(Deserialize)]
struct HostConfig {
    host: Host,
    provision: Option<Provision>,
}

#[derive(Deserialize)]
struct Host {
    name: String,
}

#[derive(Deserialize)]
struct Provision {
    scripts: Option<Vec<String>>,
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

    let scripts = cfg.provision.and_then(|p| p.scripts).unwrap_or_default();

    if scripts.is_empty() {
        println!("No provisioning scripts listed. Nothing to run.");
        println!("PSH setup complete for host '{}'.", cfg.host.name);
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

    if failures.is_empty() {
        println!("PSH setup complete for host '{}'", cfg.host.name);
    } else {
        println!(
            "PSH setup finished with warnings for host '{}':",
            cfg.host.name
        );
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
