use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use duct::cmd;
use serde::Deserialize;
use which::which;

#[derive(Deserialize)]
struct ServiceManifest {
    service: HashMap<String, ServiceConfig>,
}

#[derive(Deserialize, Clone)]
struct ServiceConfig {
    compose: String,
    project: Option<String>,
    #[serde(default)]
    setup_scripts: Vec<String>,
    #[serde(default)]
    teardown_scripts: Vec<String>,
    #[serde(default)]
    env: HashMap<String, String>,
    description: Option<String>,
}

pub fn setup_service(service: &str) -> Result<()> {
    let service_dir = locate_service_dir(service)
        .with_context(|| format!("service '{}' not found under services/", service))?;
    let config = load_service_config(&service_dir, service)?;

    for script in &config.setup_scripts {
        let resolved = resolve_script_path(script, &service_dir);
        if !resolved.exists() {
            bail!(
                "setup script not found for service '{}': {}",
                service,
                resolved.display()
            );
        }

        println!("[{}] Running setup script: {}", service, resolved.display());
        run_script(&resolved).with_context(|| {
            format!("[{}] setup script failed: {}", service, resolved.display())
        })?;
    }

    Ok(())
}

pub fn setup_services(services: &[String]) -> Result<()> {
    for svc in services {
        setup_service(svc)?;
    }
    Ok(())
}

pub fn teardown_service(service: &str) -> Result<()> {
    let service_dir = locate_service_dir(service)
        .with_context(|| format!("service '{}' not found under services/", service))?;
    let config = load_service_config(&service_dir, service)?;

    bring_service_down(service)?;

    for script in &config.teardown_scripts {
        let resolved = resolve_script_path(script, &service_dir);
        if !resolved.exists() {
            println!(
                "[{}] Teardown script missing (skipping): {}",
                service,
                resolved.display()
            );
            continue;
        }

        println!(
            "[{}] Running teardown script: {}",
            service,
            resolved.display()
        );
        if let Err(err) = run_script(&resolved) {
            eprintln!(
                "[{}] Teardown script failed ({}): {}",
                service,
                resolved.display(),
                err
            );
        }
    }

    Ok(())
}

pub fn bring_service_up(service: &str) -> Result<()> {
    let service_dir = locate_service_dir(service)
        .with_context(|| format!("service '{}' not found under services/", service))?;
    let config = load_service_config(&service_dir, service)?;

    which("docker").context("docker executable not found in PATH")?;

    let compose_path = compose_file_path(&service_dir, &config)?;
    let project = compose_project_name(service, &config);

    println!(
        "==> Starting service '{}' using compose file {}",
        service,
        compose_path.display()
    );

    let mut expr = cmd!(
        "docker",
        "compose",
        "-f",
        &compose_path,
        "-p",
        &project,
        "up",
        "-d"
    )
    .dir(&service_dir)
    .stderr_to_stdout();

    for (key, value) in &config.env {
        expr = expr.env(key, value);
    }

    expr.run()
        .with_context(|| format!("[{}] docker compose up failed", service))?;

    Ok(())
}

pub fn bring_service_down(service: &str) -> Result<()> {
    let service_dir = locate_service_dir(service)
        .with_context(|| format!("service '{}' not found under services/", service))?;
    let config = load_service_config(&service_dir, service)?;

    which("docker").context("docker executable not found in PATH")?;

    let compose_path = compose_file_path(&service_dir, &config)?;
    let project = compose_project_name(service, &config);

    println!(
        "==> Stopping service '{}' using compose file {}",
        service,
        compose_path.display()
    );

    let mut expr = cmd!(
        "docker",
        "compose",
        "-f",
        &compose_path,
        "-p",
        &project,
        "down"
    )
    .dir(&service_dir)
    .stderr_to_stdout();

    for (key, value) in &config.env {
        expr = expr.env(key, value);
    }

    expr.run()
        .with_context(|| format!("[{}] docker compose down failed", service))?;

    Ok(())
}

pub fn list_services() -> Result<()> {
    let services_root = locate_services_root()
        .with_context(|| "unable to locate services/ directory".to_string())?;

    let mut entries: Vec<_> = fs::read_dir(&services_root)
        .with_context(|| format!("failed to read {}", services_root.display()))?
        .filter_map(|entry| entry.ok())
        .filter(|entry| entry.path().is_dir())
        .collect();

    entries.sort_by_key(|entry| entry.file_name());

    if entries.is_empty() {
        println!("No services found under {}", services_root.display());
        return Ok(());
    }

    println!("Services under {}:", services_root.display());
    for entry in entries {
        let name = entry.file_name().to_string_lossy().to_string();
        let service_dir = entry.path();

        match load_service_config(&service_dir, &name) {
            Ok(config) => match service_status_with_config(&name, &service_dir, &config) {
                Ok(status) => {
                    let mut line = format!("- {:<20} {}", name, status);
                    if let Some(desc) = config.description.as_deref() {
                        line.push_str(&format!(" – {}", desc));
                    }
                    println!("{}", line);
                }
                Err(err) => println!("- {:<20} (error: {})", name, err),
            },
            Err(err) => println!("- {:<20} (error: {})", name, err),
        }
    }

    Ok(())
}

pub fn all_service_names() -> Result<Vec<String>> {
    let services_root = locate_services_root()
        .with_context(|| "unable to locate services/ directory".to_string())?;

    let mut entries: Vec<_> = fs::read_dir(&services_root)?
        .filter_map(|entry| entry.ok())
        .filter(|entry| entry.path().is_dir())
        .collect();

    entries.sort_by_key(|entry| entry.file_name());

    let names = entries
        .into_iter()
        .map(|entry| entry.file_name().to_string_lossy().to_string())
        .collect();

    Ok(names)
}

fn service_status_with_config(
    service: &str,
    service_dir: &Path,
    config: &ServiceConfig,
) -> Result<String> {
    which("docker").context("docker executable not found in PATH")?;

    let compose_path = compose_file_path(service_dir, config)?;
    let project = compose_project_name(service, config);

    let mut expr = cmd!(
        "docker",
        "compose",
        "-f",
        &compose_path,
        "-p",
        &project,
        "ps"
    )
    .dir(service_dir)
    .stderr_to_stdout();

    for (key, value) in &config.env {
        expr = expr.env(key, value);
    }

    let output = expr
        .read()
        .with_context(|| format!("[{}] failed to inspect service status", service))?;

    let running = output.lines().skip(1).any(|line| line.contains("Up"));

    Ok(if running { "running" } else { "stopped" }.to_string())
}

fn load_service_config(service_dir: &Path, service: &str) -> Result<ServiceConfig> {
    let manifest_path = service_dir.join("service.toml");
    let content = fs::read_to_string(&manifest_path)
        .with_context(|| format!("failed to read {}", manifest_path.display()))?;

    let manifest: ServiceManifest = toml::from_str(&content)
        .with_context(|| format!("failed to parse {}", manifest_path.display()))?;

    manifest.service.get(service).cloned().ok_or_else(|| {
        anyhow!(
            "service '{}' missing service.{} entry in service.toml",
            service,
            service
        )
    })
}

fn compose_file_path(service_dir: &Path, config: &ServiceConfig) -> Result<PathBuf> {
    let path = PathBuf::from(&config.compose);
    let resolved = if path.is_absolute() {
        path
    } else {
        service_dir.join(path)
    };

    resolved
        .canonicalize()
        .with_context(|| format!("compose file not found: {}", resolved.display()))
}

fn compose_project_name(service: &str, config: &ServiceConfig) -> String {
    config
        .project
        .clone()
        .unwrap_or_else(|| format!("psyched-{}", service))
}

fn run_script(path: &Path) -> Result<()> {
    cmd!("bash", path)
        .stderr_to_stdout()
        .run()
        .map(|_| ())
        .with_context(|| format!("command failed: {}", path.display()))
}

fn resolve_script_path(script: &str, service_dir: &Path) -> PathBuf {
    if let Ok(found) = which(script) {
        return found;
    }

    let requested = PathBuf::from(script);
    if requested.is_absolute() {
        return requested;
    }

    let mut candidates = Vec::new();
    candidates.push(service_dir.join(&requested));

    if let Some(repo_root) = service_dir.parent() {
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

fn locate_service_dir(service: &str) -> Option<PathBuf> {
    let services_root = locate_services_root()?;
    let candidate = services_root.join(service);
    if candidate.exists() {
        Some(candidate)
    } else {
        None
    }
}

fn locate_services_root() -> Option<PathBuf> {
    service_search_roots()
        .into_iter()
        .map(|root| root.join("services"))
        .find(|candidate| candidate.is_dir())
}

fn service_search_roots() -> Vec<PathBuf> {
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
