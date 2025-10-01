use anyhow::{Context, Result};
use duct::cmd;
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
