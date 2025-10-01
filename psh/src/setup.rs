use anyhow::{Context, Result};
use duct::cmd;
use serde::Deserialize;
use std::env;
use std::fs;
use std::fs::OpenOptions;
use std::io::{BufRead, BufReader, Write};
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

pub fn setup_env() -> Result<()> {
    // Determine the ROS distribution (from environment or default to "kilted")
    let ros_distro = env::var("ROS_DISTRO").unwrap_or_else(|_| "kilted".to_string());

    // Get the home directory
    let home_dir = directories::BaseDirs::new()
        .context("failed to determine home directory")?
        .home_dir()
        .to_path_buf();

    let bashrc_path = home_dir.join(".bashrc");

    // The function to add - it will source ROS2 setup when called
    let function_block = format!(
        "psyched() {{\n    source /opt/ros/{}/setup.bash\n}}",
        ros_distro
    );

    // Check if the function already exists
    let mut function_exists = false;
    if bashrc_path.exists() {
        let file = fs::File::open(&bashrc_path).context("failed to open ~/.bashrc")?;
        let reader = BufReader::new(file);

        for line in reader.lines() {
            if let Ok(line) = line {
                if line.contains("psyched()") {
                    function_exists = true;
                    break;
                }
            }
        }
    }

    // Add the function if it doesn't exist
    if !function_exists {
        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&bashrc_path)
            .context("failed to open ~/.bashrc for writing")?;

        writeln!(file, "\n# Added by psh env")?;
        writeln!(file, "{}", function_block)?;
        writeln!(file, "psyched  # Auto-activate ROS 2 environment")?;

        println!(
            "✓ Added 'psyched' shell function to {}/.bashrc",
            home_dir.display()
        );
        println!(
            "Appended the following lines to {}/.bashrc:",
            home_dir.display()
        );
        println!("# Added by psh env");
        println!("psyched() {{");
        println!("    source /opt/ros/{}/setup.bash", ros_distro);
        println!("}}",);
        println!("psyched  # Auto-activate ROS 2 environment\n");
        println!(
            "This will automatically source /opt/ros/{}/setup.bash for new interactive shells.",
            ros_distro
        );
        println!("To activate the ROS 2 environment in your CURRENT shell, run:");
        println!("  source ~/.bashrc");
        println!("or just run:");
        println!("  psyched\n");
        println!(
            "If you want a different ROS distribution, re-run this command with ROS_DISTRO set, e.g."
        );
        println!("  ROS_DISTRO=humble psh env");
    } else {
        println!(
            "✓ 'psyched' function already exists in {}/.bashrc",
            home_dir.display()
        );
        println!("It will auto-activate the ROS 2 environment for new shells.");
        println!("To activate it in the current shell, run: source ~/.bashrc  (or run 'psyched')");
        println!(
            "To change which ROS distribution is sourced, remove or edit the 'psyched' block in ~/.bashrc and re-run with ROS_DISTRO set."
        );
    }

    println!("\nDone. The ROS 2 environment will be active in any new interactive shells.");
    println!("To activate it in this current terminal, run:");
    println!("  source ~/.bashrc");
    println!("or run:");
    println!("  psyched\n");
    println!("If you want to inspect what was added, run:");
    println!("  tail -n 10 ~/.bashrc");

    Ok(())
}
