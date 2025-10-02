use anyhow::{Context, Result, anyhow, bail};
use directories::ProjectDirs;
use duct::cmd;
use regex::Regex;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::OnceLock;
use walkdir::WalkDir;
use which::which;

#[cfg(target_family = "unix")]
use std::os::unix::fs::symlink;
#[cfg(target_family = "windows")]
use std::os::windows::fs::{symlink_dir, symlink_file};

const APP_QUALIFIER: &str = "io";
const APP_ORGANIZATION: &str = "rizzlington";
const APP_NAME: &str = "psh";

#[derive(Deserialize)]
struct ModuleManifest {
    unit: HashMap<String, UnitConfig>,
}

#[derive(Deserialize, Clone)]
struct UnitConfig {
    #[serde(default)]
    apt: Vec<String>,
    #[serde(default)]
    pip: Vec<String>,
    patches: Option<Vec<String>>,
    launch: Option<String>,
    shutdown: Option<String>,
    pilot_control: Option<String>,
    git: Option<Vec<GitRepo>>,
    ros: Option<RosBuildConfig>,
}

#[derive(Deserialize, Clone)]
struct GitRepo {
    url: String,
    branch: Option<String>,
    path: Option<String>,
}

#[derive(Deserialize, Clone, Default)]
struct RosBuildConfig {
    workspace: Option<String>,
    #[serde(default)]
    packages: Vec<String>,
    #[serde(default)]
    build_args: Vec<String>,
    #[serde(default)]
    skip_rosdep_keys: Vec<String>,
    #[serde(default)]
    skip_rosdep: bool,
    #[serde(default)]
    skip_colcon: bool,
}

pub fn setup_module(module: &str) -> Result<()> {
    setup_modules(&[module.to_string()])
}

pub fn setup_modules(modules: &[String]) -> Result<()> {
    if modules.is_empty() {
        return Ok(());
    }

    let mut plans: Vec<(String, PathBuf, UnitConfig)> = Vec::new();
    let mut apt_requirements: Vec<String> = Vec::new();
    let mut pip_requirements: Vec<String> = Vec::new();

    for module in modules {
        let module_dir = locate_module_dir(module)
            .with_context(|| format!("module '{}' not found under modules/", module))?;

        let unit_config = load_unit_config(&module_dir, module)?;

        apt_requirements.extend(unit_config.apt.iter().cloned());
        pip_requirements.extend(unit_config.pip.iter().cloned());

        plans.push((module.clone(), module_dir, unit_config));
    }

    let apt_unique = dedupe_preserve_order(apt_requirements);
    if !apt_unique.is_empty() {
        let label = format!("batch(setup:{})", modules.join(","));
        install_apt_packages(&label, &apt_unique)?;
    }

    let pip_unique = dedupe_preserve_order(pip_requirements);
    if !pip_unique.is_empty() {
        let label = format!("batch(setup:{})", modules.join(","));
        install_pip_packages(&label, &pip_unique)?;
    }

    for (module, module_dir, unit_config) in plans {
        if let Some(repos) = unit_config.git.as_ref() {
            sync_git_repos(&module, &module_dir, repos)?;
        }

        if let Some(patches) = unit_config.patches.as_ref() {
            run_patch_scripts(&module, &module_dir, patches);
        }

        prepare_ros_workspace(&module, &module_dir, unit_config.ros.as_ref())?;

        setup_module_internal(&module, &module_dir, true)?;
    }

    Ok(())
}

pub fn teardown_module(module: &str) -> Result<()> {
    let module_dir = locate_module_dir(module)
        .with_context(|| format!("module '{}' not found under modules/", module))?;

    teardown_module_internal(module, &module_dir, true)
}

fn setup_module_internal(module: &str, module_dir: &Path, strict: bool) -> Result<()> {
    if strict {
        println!("==> Setting up module '{}'", module);
    }

    let overlay_root = match locate_pilot_frontend(module_dir) {
        Ok(path) => path,
        Err(err) => {
            if strict {
                return Err(err);
            } else {
                eprintln!("[{}] Pilot overlay skipped: {}", module, err);
                return Ok(());
            }
        }
    };

    link_pilot_assets(module, module_dir, &overlay_root)?;
    rebuild_pilot_manifest(&overlay_root)?;
    Ok(())
}

fn sync_git_repos(module: &str, module_dir: &Path, repos: &[GitRepo]) -> Result<()> {
    if repos.is_empty() {
        return Ok(());
    }

    let repo_root = module_dir
        .parent()
        .and_then(|modules_dir| modules_dir.parent())
        .ok_or_else(|| {
            anyhow!(
                "unable to determine repository root for {}",
                module_dir.display()
            )
        })?;

    let src_dir = repo_root.join("src");
    fs::create_dir_all(&src_dir)
        .with_context(|| format!("failed to ensure {} exists", src_dir.display()))?;

    for repo in repos {
        let destination = if let Some(path) = &repo.path {
            src_dir.join(path)
        } else {
            let name = derive_repo_dirname(&repo.url)?;
            src_dir.join(name)
        };

        let branch_desc = repo.branch.as_deref().unwrap_or("<default>");
        println!(
            "[{}] Syncing git repo {} (branch {}) into {}",
            module,
            repo.url,
            branch_desc,
            destination.display()
        );

        if destination.exists() {
            if !destination.is_dir() {
                bail!(
                    "target path {} exists and is not a directory",
                    destination.display()
                );
            }

            if !destination.join(".git").is_dir() {
                bail!(
                    "target path {} exists but is not a git repository",
                    destination.display()
                );
            }

            if let Some(branch) = repo.branch.as_deref() {
                cmd!("git", "-C", &destination, "fetch", "origin", branch)
                    .stderr_to_stdout()
                    .run()
                    .with_context(|| {
                        format!("failed to fetch branch {} for {}", branch, repo.url)
                    })?;

                cmd!("git", "-C", &destination, "checkout", branch)
                    .stderr_to_stdout()
                    .run()
                    .with_context(|| {
                        format!("failed to checkout branch {} for {}", branch, repo.url)
                    })?;

                cmd!(
                    "git",
                    "-C",
                    &destination,
                    "reset",
                    "--hard",
                    format!("origin/{}", branch)
                )
                .stderr_to_stdout()
                .run()
                .with_context(|| format!("failed to reset branch {} for {}", branch, repo.url))?;
            } else {
                cmd!("git", "-C", &destination, "fetch", "--all")
                    .stderr_to_stdout()
                    .run()
                    .with_context(|| format!("failed to fetch updates for {}", repo.url))?;

                cmd!("git", "-C", &destination, "pull", "--ff-only")
                    .stderr_to_stdout()
                    .run()
                    .with_context(|| format!("failed to pull updates for {}", repo.url))?;
            }
        } else {
            if let Some(parent) = destination.parent() {
                fs::create_dir_all(parent).with_context(|| {
                    format!("failed to ensure parent directory {}", parent.display())
                })?;
            }

            if let Some(branch) = repo.branch.as_deref() {
                cmd!(
                    "git",
                    "clone",
                    "--branch",
                    branch,
                    "--single-branch",
                    &repo.url,
                    &destination
                )
                .stderr_to_stdout()
                .run()
                .with_context(|| format!("failed to clone {} (branch {})", repo.url, branch))?;
            } else {
                cmd!("git", "clone", &repo.url, &destination)
                    .stderr_to_stdout()
                    .run()
                    .with_context(|| format!("failed to clone {}", repo.url))?;
            }
        }
    }

    Ok(())
}

fn prepare_ros_workspace(
    module: &str,
    module_dir: &Path,
    ros_config: Option<&RosBuildConfig>,
) -> Result<()> {
    let Some(config) = ros_config else {
        return Ok(());
    };

    let repo_root = module_dir
        .parent()
        .and_then(|modules_dir| modules_dir.parent())
        .ok_or_else(|| {
            anyhow!(
                "unable to determine repository root for {}",
                module_dir.display()
            )
        })?;

    let workspace_rel = config.workspace.as_deref().unwrap_or(".");
    let workspace_dir = repo_root.join(workspace_rel);

    if !workspace_dir.exists() {
        bail!(
            "ROS workspace {} not found for module '{}'",
            workspace_dir.display(),
            module
        );
    }

    let src_dir = workspace_dir.join("src");
    if !src_dir.is_dir() {
        bail!(
            "ROS workspace {} missing src/ directory for module '{}'",
            workspace_dir.display(),
            module
        );
    }

    if !config.skip_rosdep {
        which("rosdep").with_context(|| {
            "rosdep binary not found in PATH; install rosdep or add it to PATH".to_string()
        })?;

        println!(
            "[{}] Running rosdep install in {}",
            module,
            workspace_dir.display()
        );

        let mut rosdep_args = vec![
            "install".to_string(),
            "--from-paths".to_string(),
            src_dir.to_string_lossy().into_owned(),
            "--ignore-src".to_string(),
            "-r".to_string(),
            "-y".to_string(),
        ];

        if !config.skip_rosdep_keys.is_empty() {
            rosdep_args.push("--skip-keys".to_string());
            rosdep_args.push(config.skip_rosdep_keys.join(" "));
        }

        cmd("rosdep", rosdep_args)
            .dir(&workspace_dir)
            .stderr_to_stdout()
            .run()
            .with_context(|| format!("[{}] rosdep install failed", module))?;
    } else {
        println!("[{}] Skipping rosdep install (skip_rosdep = true)", module);
    }

    if config.skip_colcon {
        println!("[{}] Skipping colcon build (skip_colcon = true)", module);
        return Ok(());
    }

    which("colcon").with_context(|| {
        "colcon binary not found in PATH; ensure colcon is installed".to_string()
    })?;

    let mut colcon_args = vec!["build".to_string(), "--symlink-install".to_string()];

    if !config.build_args.is_empty() {
        colcon_args.extend(config.build_args.clone());
    }

    if !config.packages.is_empty() {
        colcon_args.push("--packages-select".to_string());
        colcon_args.extend(config.packages.iter().cloned());
    }

    // TODO: Only run this once `psh mod setup` happens
    // println!(
    //     "[{}] Running colcon build in {}",
    //     module,
    //     workspace_dir.display()
    // );

    // cmd("colcon", colcon_args)
    //     .dir(&workspace_dir)
    //     .stderr_to_stdout()
    //     .run()
    //     .with_context(|| format!("[{}] colcon build failed", module))?;

    Ok(())
}

fn install_apt_packages(label: &str, packages: &[String]) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    which("apt-get").with_context(|| {
        format!(
            "[{}] apt-get not found in PATH; unable to install apt packages",
            label
        )
    })?;

    let mut expanded = Vec::with_capacity(packages.len());
    for pkg in packages {
        let resolved = expand_env_placeholders(pkg).with_context(|| {
            format!(
                "[{}] failed to expand environment variables in '{}'",
                label, pkg
            )
        })?;

        let trimmed = resolved.trim();
        if trimmed.is_empty() {
            bail!(
                "[{}] expanded apt package '{}' resolved to an empty string",
                label,
                pkg
            );
        }

        expanded.push(trimmed.to_string());
    }

    println!(
        "[{}] Installing apt packages: {}",
        label,
        expanded.join(", ")
    );

    let use_sudo = which("sudo").is_ok();

    let mut install_args: Vec<String> = vec!["install".into(), "-y".into()];
    install_args.extend(expanded.iter().cloned());

    let install_expression = if use_sudo {
        let mut args = Vec::with_capacity(1 + install_args.len());
        args.push("apt-get".to_string());
        args.extend(install_args.iter().cloned());
        cmd("sudo", args)
    } else {
        cmd("apt-get", install_args.clone())
    };

    install_expression
        .env("DEBIAN_FRONTEND", "noninteractive")
        .stderr_to_stdout()
        .run()
        .with_context(|| format!("[{}] apt package installation failed", label))?;

    Ok(())
}

fn install_pip_packages(label: &str, packages: &[String]) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    let pip_cmd = which("pip3")
        .or_else(|_| which("pip"))
        .with_context(|| format!("[{}] pip executable not found (pip3 or pip)", label))?;
    let pip_program = pip_cmd.to_string_lossy().into_owned();

    let mut expanded = Vec::with_capacity(packages.len());
    for pkg in packages {
        let resolved = expand_env_placeholders(pkg).with_context(|| {
            format!(
                "[{}] failed to expand environment variables in '{}'",
                label, pkg
            )
        })?;

        let trimmed = resolved.trim();
        if trimmed.is_empty() {
            bail!(
                "[{}] expanded pip package '{}' resolved to an empty string",
                label,
                pkg
            );
        }

        expanded.push(trimmed.to_string());
    }

    println!(
        "[{}] Installing pip packages: {}",
        label,
        expanded.join(", ")
    );

    let mut args = Vec::with_capacity(1 + expanded.len());
    args.push("install".to_string());
    args.extend(expanded.iter().cloned());

    cmd(pip_program, args)
        .stderr_to_stdout()
        .run()
        .with_context(|| format!("[{}] pip package installation failed", label))?;

    Ok(())
}

fn expand_env_placeholders(input: &str) -> Result<String> {
    static ENV_VAR_REGEX: OnceLock<Regex> = OnceLock::new();

    let regex = ENV_VAR_REGEX.get_or_init(|| {
        Regex::new(r"\$\{([A-Za-z0-9_]+)\}").expect("valid environment variable pattern")
    });

    let mut result = String::with_capacity(input.len());
    let mut last_index = 0;

    for captures in regex.captures_iter(input) {
        let m = captures
            .get(0)
            .expect("captures_iter should yield a full match");
        result.push_str(&input[last_index..m.start()]);

        let var_name = captures
            .get(1)
            .expect("captures_iter should yield capture group 1")
            .as_str();

        let value = env::var(var_name)
            .with_context(|| format!("environment variable {} is not set", var_name))?;

        result.push_str(&value);
        last_index = m.end();
    }

    result.push_str(&input[last_index..]);
    Ok(result)
}

fn dedupe_preserve_order(items: Vec<String>) -> Vec<String> {
    let mut seen = HashSet::new();
    let mut result = Vec::new();
    for item in items {
        if seen.insert(item.clone()) {
            result.push(item);
        }
    }
    result
}

fn run_patch_scripts(module: &str, module_dir: &Path, scripts: &[String]) {
    for script in scripts {
        println!("[{}] Running patch script: {}", module, script);
        let resolved = resolve_script_path(script, module_dir);
        if !resolved.exists() {
            eprintln!("[{}] Patch script missing: {}", module, resolved.display());
            continue;
        }

        if let Err(err) = run_script(&resolved) {
            eprintln!(
                "[{}] Patch script failed ({}): {}",
                module,
                resolved.display(),
                err
            );
        } else {
            println!("[{}] Patch script complete: {}", module, resolved.display());
        }
    }
}

fn derive_repo_dirname(url: &str) -> Result<String> {
    let trimmed = url.trim_end_matches('/');
    let candidate = trimmed
        .rsplit('/')
        .next()
        .ok_or_else(|| anyhow!("unable to derive repository name from {}", url))?;
    let name = candidate.trim_end_matches(".git");
    if name.is_empty() {
        bail!("repository name resolved to empty string for {}", url);
    }
    Ok(name.to_string())
}

fn teardown_module_internal(module: &str, module_dir: &Path, strict: bool) -> Result<()> {
    if strict {
        println!("==> Tearing down module '{}'", module);
    }

    let overlay_root = match locate_pilot_frontend(module_dir) {
        Ok(path) => path,
        Err(err) => {
            if strict {
                return Err(err);
            } else {
                eprintln!("[{}] Pilot overlay skip during teardown: {}", module, err);
                return Ok(());
            }
        }
    };

    unlink_pilot_assets(module, module_dir, &overlay_root)?;
    rebuild_pilot_manifest(&overlay_root)?;
    Ok(())
}

pub fn bring_module_up(module: &str) -> Result<()> {
    let module_dir = locate_module_dir(module)
        .with_context(|| format!("module '{}' not found under modules/", module))?;

    let unit_config = load_unit_config(&module_dir, module)?;

    println!("==> Loading module '{}'", module);

    if let Some(control) = unit_config.pilot_control.as_deref() {
        println!("[{}] Pilot control component: {}", module, control);
    }

    let launch = unit_config
        .launch
        .as_ref()
        .ok_or_else(|| anyhow!("module '{}' has no launch command", module))?;

    ensure_not_already_running(module)?;

    println!("[{}] Launching service: {}", module, launch);
    let launch_path = resolve_script_path(launch, &module_dir);
    if !launch_path.exists() {
        return Err(anyhow!(
            "launch script not found: {}",
            launch_path.display()
        ));
    }

    let mut cmd = Command::new("bash");
    cmd.arg(&launch_path)
        .stdin(Stdio::null())
        .stdout(Stdio::inherit())
        .stderr(Stdio::inherit());

    let child = cmd
        .spawn()
        .with_context(|| format!("failed to launch {}", launch_path.display()))?;

    let pid = child.id();

    println!("[{}] Launch started with PID {}", module, pid);

    record_pid(module, pid)?;

    // Detach so the child keeps running after this function returns.
    std::mem::forget(child);

    Ok(())
}

pub fn bring_module_down(module: &str) -> Result<()> {
    let module_dir = locate_module_dir(module)
        .with_context(|| format!("module '{}' not found under modules/", module))?;

    let unit_config = load_unit_config(&module_dir, module)?;

    println!("==> Shutting down module '{}'", module);

    if let Some(shutdown) = unit_config.shutdown.as_ref() {
        let shutdown_path = resolve_script_path(shutdown, &module_dir);
        if shutdown_path.exists() {
            println!("[{}] Running shutdown script: {}", module, shutdown);
            if let Err(err) = run_script(&shutdown_path) {
                eprintln!(
                    "[{}] Shutdown script failed ({}): {}",
                    module,
                    shutdown_path.display(),
                    err
                );
            } else {
                println!(
                    "[{}] Shutdown script complete: {}",
                    module,
                    shutdown_path.display()
                );
            }
        } else {
            eprintln!(
                "[{}] Shutdown script missing: {}",
                module,
                shutdown_path.display()
            );
        }
    } else {
        println!(
            "[{}] No shutdown command configured; nothing to do.",
            module
        );
    }

    if let Some(pid) = read_pid(module)? {
        if is_pid_running(pid) {
            println!(
                "[{}] Launch process (pid {}) still reported as running; verify shutdown.",
                module, pid
            );
        }
    }

    clear_pid(module);

    Ok(())
}

pub fn list_modules() -> Result<()> {
    let modules_root =
        locate_modules_root().with_context(|| "unable to locate modules/ directory".to_string())?;

    let mut entries: Vec<_> = fs::read_dir(&modules_root)
        .with_context(|| format!("failed to read {}", modules_root.display()))?
        .filter_map(|entry| entry.ok())
        .filter(|entry| entry.path().is_dir())
        .collect();

    entries.sort_by_key(|entry| entry.file_name());

    if entries.is_empty() {
        println!("No modules found under {}", modules_root.display());
        return Ok(());
    }

    println!("Modules under {}:", modules_root.display());
    for entry in entries {
        let name = entry.file_name().to_string_lossy().to_string();
        let status = module_status(&name)?;
        println!("- {:<20} {}", name, status);
    }

    Ok(())
}

/// Return a Vec of all module directory names under the repository's modules/ directory.
pub fn all_module_names() -> Result<Vec<String>> {
    let modules_root =
        locate_modules_root().with_context(|| "unable to locate modules/ directory".to_string())?;

    let mut entries: Vec<_> = fs::read_dir(&modules_root)?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .collect();

    entries.sort_by_key(|e| e.file_name());

    let names = entries
        .into_iter()
        .map(|e| e.file_name().to_string_lossy().to_string())
        .collect();

    Ok(names)
}

fn module_status(module: &str) -> Result<String> {
    match read_pid(module)? {
        Some(pid) if is_pid_running(pid) => Ok(format!("running (pid {})", pid)),
        Some(_) => {
            clear_pid(module);
            Ok("stopped".to_string())
        }
        None => Ok("stopped".to_string()),
    }
}

fn link_pilot_assets(module: &str, module_dir: &Path, overlay_root: &Path) -> Result<()> {
    let pilot_dir = module_dir.join("pilot");
    if !pilot_dir.exists() {
        println!(
            "[{}] No pilot overlay directory; skipping pilot asset linking.",
            module
        );
        return Ok(());
    }

    println!(
        "[{}] Linking pilot assets from {} into {}",
        module,
        pilot_dir.display(),
        overlay_root.display()
    );

    for entry in WalkDir::new(&pilot_dir) {
        let entry = entry?;
        let path = entry.path();

        if path == pilot_dir {
            continue;
        }

        let relative = path
            .strip_prefix(&pilot_dir)
            .with_context(|| format!("failed to strip prefix for {}", path.display()))?;

        let destination = overlay_root.join(relative);

        if entry.file_type().is_dir() {
            fs::create_dir_all(&destination)
                .with_context(|| format!("failed to ensure directory {}", destination.display()))?;
            continue;
        }

        if let Some(parent) = destination.parent() {
            fs::create_dir_all(parent).with_context(|| {
                format!("failed to create destination parent {}", parent.display())
            })?;
        }

        if let Ok(existing) = fs::symlink_metadata(&destination) {
            if existing.file_type().is_symlink() {
                let target = fs::read_link(&destination).with_context(|| {
                    format!("failed to read existing symlink {}", destination.display())
                })?;
                if target == path {
                    // Already linked as desired; nothing more to do for this entry.
                    continue;
                }
                fs::remove_file(&destination).with_context(|| {
                    format!(
                        "failed to replace existing symlink {}",
                        destination.display()
                    )
                })?;
            } else {
                bail!(
                    "refusing to overwrite existing non-symlink destination {}",
                    destination.display()
                );
            }
        }

        create_symlink(path, &destination).with_context(|| {
            format!(
                "failed to create symlink {} -> {}",
                destination.display(),
                path.display()
            )
        })?;
    }

    Ok(())
}

fn unlink_pilot_assets(module: &str, module_dir: &Path, overlay_root: &Path) -> Result<()> {
    let pilot_dir = module_dir.join("pilot");
    if !pilot_dir.exists() {
        println!(
            "[{}] No pilot overlay directory; nothing to unlink.",
            module
        );
        return Ok(());
    }

    println!(
        "[{}] Removing pilot asset links from {}",
        module,
        overlay_root.display()
    );

    let mut cleanup_dirs: HashSet<PathBuf> = HashSet::new();

    for entry in WalkDir::new(&overlay_root) {
        let entry = entry?;
        let path = entry.path();

        if entry.file_type().is_symlink() {
            let target = fs::read_link(path)
                .with_context(|| format!("failed to read symlink {}", path.display()))?;

            if target.starts_with(&pilot_dir) {
                fs::remove_file(path)
                    .with_context(|| format!("failed to remove symlink {}", path.display()))?;

                let mut ancestor = path.parent();
                while let Some(parent) = ancestor {
                    if parent == overlay_root {
                        break;
                    }
                    cleanup_dirs.insert(parent.to_path_buf());
                    ancestor = parent.parent();
                }
            }
        }
    }

    let mut dirs: Vec<PathBuf> = cleanup_dirs.into_iter().collect();
    dirs.sort_by(|a, b| b.components().count().cmp(&a.components().count()));

    for dir in dirs {
        if dir.exists() && dir_is_empty(&dir)? {
            fs::remove_dir(&dir)
                .with_context(|| format!("failed to remove empty directory {}", dir.display()))?;
        }
    }

    Ok(())
}

fn locate_pilot_frontend(module_dir: &Path) -> Result<PathBuf> {
    let modules_root = module_dir.parent().ok_or_else(|| {
        anyhow!(
            "unable to determine modules root for {}",
            module_dir.display()
        )
    })?;

    let frontend = modules_root.join("pilot").join("frontend");
    if frontend.is_dir() {
        Ok(frontend)
    } else {
        Err(anyhow!(
            "pilot frontend not found at {}; run pilot module bootstrap first",
            frontend.display()
        ))
    }
}

#[cfg(target_family = "unix")]
fn create_symlink(src: &Path, dest: &Path) -> std::io::Result<()> {
    symlink(src, dest)
}

#[cfg(target_family = "windows")]
fn create_symlink(src: &Path, dest: &Path) -> std::io::Result<()> {
    if src.is_dir() {
        symlink_dir(src, dest)
    } else {
        symlink_file(src, dest)
    }
}

fn dir_is_empty(path: &Path) -> Result<bool> {
    let mut entries = fs::read_dir(path)
        .with_context(|| format!("failed to read directory {}", path.display()))?;

    while let Some(entry) = entries.next() {
        entry?;
        return Ok(false);
    }

    Ok(true)
}

fn rebuild_pilot_manifest(frontend_root: &Path) -> Result<()> {
    let mut routes = gather_manifest_entries(frontend_root, "routes")?;
    let mut islands = gather_manifest_entries(frontend_root, "islands")?;

    routes.sort();
    islands.sort();

    let mut buffer = String::new();
    buffer.push_str("import config from \"./fresh.config.ts\";\n");

    for (idx, route) in routes.iter().enumerate() {
        buffer.push_str(&format!("import * as ${} from \"./{}\";\n", idx, route));
    }

    for (idx, island) in islands.iter().enumerate() {
        buffer.push_str(&format!("import * as $I{} from \"./{}\";\n", idx, island));
    }

    buffer.push_str("import { type Manifest } from \"$fresh/server.ts\";\n\n");
    buffer.push_str("const manifest: Manifest = {\n    routes: {\n");

    for (idx, route) in routes.iter().enumerate() {
        buffer.push_str(&format!("        \"./{}\": ${},\n", route, idx));
    }

    buffer.push_str("    },\n    islands: {\n");

    for (idx, island) in islands.iter().enumerate() {
        buffer.push_str(&format!("        \"./{}\": $I{},\n", island, idx));
    }

    buffer.push_str("    },\n    baseUrl: import.meta.url,\n};\n\nexport default manifest;\nexport { config };\n");

    let path = frontend_root.join("fresh.gen.ts");
    fs::write(&path, buffer).with_context(|| format!("failed to write {}", path.display()))?;

    Ok(())
}

fn gather_manifest_entries(frontend_root: &Path, subdir: &str) -> Result<Vec<String>> {
    let dir = frontend_root.join(subdir);
    if !dir.is_dir() {
        return Ok(Vec::new());
    }

    let mut entries = Vec::new();

    for entry in WalkDir::new(&dir) {
        let entry = entry?;
        if entry.file_type().is_dir() {
            continue;
        }

        let path = entry.path();
        if !is_supported_source(path) {
            continue;
        }

        let relative = path
            .strip_prefix(frontend_root)
            .with_context(|| format!("failed to relativise {}", path.display()))?;

        entries.push(relative.to_string_lossy().replace('\\', "/"));
    }

    Ok(entries)
}

fn is_supported_source(path: &Path) -> bool {
    match path.extension().and_then(|ext| ext.to_str()) {
        Some("ts") | Some("tsx") | Some("js") | Some("jsx") => true,
        _ => false,
    }
}

fn load_unit_config(module_dir: &Path, module: &str) -> Result<UnitConfig> {
    let manifest_path = module_dir.join("module.toml");
    let content = fs::read_to_string(&manifest_path)
        .with_context(|| format!("failed to read {}", manifest_path.display()))?;

    let manifest: ModuleManifest = toml::from_str(&content)
        .with_context(|| format!("failed to parse {}", manifest_path.display()))?;

    manifest
        .unit
        .get(module)
        .cloned()
        .ok_or_else(|| anyhow!("module '{}' missing unit.{} in module.toml", module, module))
}

fn resolve_script_path(script: &str, module_dir: &Path) -> PathBuf {
    if let Ok(found) = which(script) {
        return found;
    }

    let requested = PathBuf::from(script);
    if requested.is_absolute() {
        return requested;
    }

    let mut candidates = Vec::new();
    candidates.push(module_dir.join(&requested));

    if let Some(repo_root) = module_dir.parent() {
        candidates.push(repo_root.join(&requested));
    }

    if let Ok(cwd) = std::env::current_dir() {
        candidates.push(cwd.join(&requested));
    }

    for candidate in candidates {
        if candidate.exists() {
            return candidate;
        }
    }

    requested
}

fn run_script(path: &Path) -> Result<()> {
    cmd!("bash", path)
        .stderr_to_stdout()
        .run()
        .map(|_| ())
        .with_context(|| format!("command failed: {}", path.display()))
}

fn locate_module_dir(module: &str) -> Option<PathBuf> {
    let modules_root = locate_modules_root()?;
    let candidate = modules_root.join(module);
    if candidate.exists() {
        Some(candidate)
    } else {
        None
    }
}

fn locate_modules_root() -> Option<PathBuf> {
    module_search_roots()
        .into_iter()
        .map(|root| root.join("modules"))
        .find(|candidate| candidate.is_dir())
}

fn module_search_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();

    if let Ok(cwd) = std::env::current_dir() {
        roots.push(cwd);
    }

    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            roots.push(dir.to_path_buf());
            if let Some(parent) = dir.parent() {
                roots.push(parent.to_path_buf());
            }
        }
    }

    roots
}

fn ensure_not_already_running(module: &str) -> Result<()> {
    if let Some(pid) = read_pid(module)? {
        if is_pid_running(pid) {
            bail!("module '{}' already running with pid {}", module, pid);
        } else {
            clear_pid(module);
        }
    }
    Ok(())
}

fn record_pid(module: &str, pid: u32) -> Result<()> {
    let path = pid_file_path(module)?;
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(&path, pid.to_string()).with_context(|| format!("failed to write {}", path.display()))
}

fn read_pid(module: &str) -> Result<Option<u32>> {
    let path = pid_file_path(module)?;
    if !path.exists() {
        return Ok(None);
    }

    let contents =
        fs::read_to_string(&path).with_context(|| format!("failed to read {}", path.display()))?;
    let pid = contents.trim().parse().ok();
    Ok(pid)
}

fn clear_pid(module: &str) {
    if let Ok(path) = pid_file_path(module) {
        let _ = fs::remove_file(path);
    }
}

fn pid_file_path(module: &str) -> Result<PathBuf> {
    let dir = state_root()?;
    Ok(dir.join(format!("{}.pid", module)))
}

fn state_root() -> Result<PathBuf> {
    if let Some(project_dirs) = ProjectDirs::from(APP_QUALIFIER, APP_ORGANIZATION, APP_NAME) {
        if let Some(runtime) = project_dirs.runtime_dir() {
            let path = runtime.join("modules");
            fs::create_dir_all(&path)?;
            return Ok(path);
        }

        let data = project_dirs.data_local_dir().join("modules");
        fs::create_dir_all(&data)?;
        return Ok(data);
    }

    let fallback = std::env::temp_dir().join("psh-modules");
    fs::create_dir_all(&fallback)?;
    Ok(fallback)
}

fn is_pid_running(pid: u32) -> bool {
    #[cfg(target_family = "unix")]
    {
        Path::new(&format!("/proc/{}", pid)).exists()
    }

    #[cfg(not(target_family = "unix"))]
    {
        // Fallback: try sending signal 0 via std::process::Command (best effort)
        // On non-Unix systems we conservatively assume not running.
        let _ = pid;
        false
    }
}
