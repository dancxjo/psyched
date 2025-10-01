use anyhow::{Context, Result, anyhow, bail};
use directories::ProjectDirs;
use duct::cmd;
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
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
    patches: Option<Vec<String>>,
    launch: Option<String>,
    shutdown: Option<String>,
    pilot_control: Option<String>,
    git: Option<Vec<GitRepo>>,
}

#[derive(Deserialize, Clone)]
struct GitRepo {
    url: String,
    branch: Option<String>,
    path: Option<String>,
}

pub fn setup_module(module: &str) -> Result<()> {
    let module_dir = locate_module_dir(module)
        .with_context(|| format!("module '{}' not found under modules/", module))?;

    let unit_config = load_unit_config(&module_dir, module)?;

    if let Some(repos) = unit_config.git.as_ref() {
        sync_git_repos(module, &module_dir, repos)?;
    }

    setup_module_internal(module, &module_dir, true)
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

    if let Some(repos) = unit_config.git.as_ref() {
        sync_git_repos(module, &module_dir, repos)?;
    }

    setup_module_internal(module, &module_dir, false)?;

    if let Some(control) = unit_config.pilot_control.as_deref() {
        println!("[{}] Pilot control component: {}", module, control);
    }

    if let Some(patches) = &unit_config.patches {
        for script in patches {
            println!("[{}] Running patch script: {}", module, script);
            let resolved = resolve_script_path(script, &module_dir);
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

    teardown_module_internal(module, &module_dir, false)?;

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
