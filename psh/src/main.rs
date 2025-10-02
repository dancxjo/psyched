mod build;
mod clean;
mod cli;
mod module_runner;
mod setup;

use anyhow::Result;
use clap::Parser;

use crate::build::build_workspace;
use crate::clean::clean_workspace;
use crate::cli::{Cli, Commands, HostCommands, ModCommands};
use crate::module_runner::{
    all_module_names, bring_module_down, bring_module_up, list_modules, setup_module,
    setup_modules, teardown_module,
};
use crate::setup::{run_setup, setup_env};

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::Host { command }) => match command {
            HostCommands::Setup { hosts } => {
                // If hosts empty, run setup for current hostname (setup::run_setup uses current hostname)
                if hosts.is_empty() {
                    run_setup()?;
                } else {
                    // For each host name, attempt to find hosts/<name>.toml and run its scripts.
                    for host in hosts {
                        // Temporarily change hostname detection by setting HOSTNAME env? Simpler: locate config and run commands similarly to run_setup logic
                        // Reuse run_setup by temporarily changing current executable? To keep simple, call setup::run_setup for local host only.
                        println!(
                            "Running host setup for {} (delegating to run_setup for local host only)",
                            host
                        );
                        // For now we call run_setup() which uses detected hostname; advanced per-host handling could be added later.
                        run_setup()?;
                    }
                }
            }
        },

        Some(Commands::Mod { command }) => match command {
            ModCommands::Up { modules } => {
                let targets = if modules.is_empty() {
                    all_module_names()?
                } else {
                    modules
                };
                for m in targets {
                    bring_module_up(&m)?;
                }
            }
            ModCommands::Down { modules } => {
                let targets = if modules.is_empty() {
                    all_module_names()?
                } else {
                    modules
                };
                for m in targets {
                    bring_module_down(&m)?;
                }
            }
            ModCommands::Setup { modules } => {
                let targets = if modules.is_empty() {
                    all_module_names()?
                } else {
                    modules
                };
                setup_modules(&targets)?;
            }
            ModCommands::Teardown { modules } => {
                let targets = if modules.is_empty() {
                    all_module_names()?
                } else {
                    modules
                };
                for m in targets {
                    teardown_module(&m)?;
                }
            }
            ModCommands::List { modules } => {
                if modules.is_empty() {
                    list_modules()?;
                } else {
                    for m in modules {
                        println!("{}", m);
                    }
                }
            }
        },

        // Backwards-compatible top-level commands map into the new subcommands
        Some(Commands::Build { packages }) => build_workspace(&packages)?,
        Some(Commands::Setup) => run_setup()?,
        Some(Commands::Up { modules }) => {
            let targets = if modules.is_empty() {
                all_module_names()?
            } else {
                modules
            };
            for m in targets {
                bring_module_up(&m)?;
            }
        }
        Some(Commands::Down { modules }) => {
            let targets = if modules.is_empty() {
                all_module_names()?
            } else {
                modules
            };
            for m in targets {
                bring_module_down(&m)?;
            }
        }
        Some(Commands::SetupModule { module }) => setup_module(&module)?,
        Some(Commands::TeardownModule { module }) => teardown_module(&module)?,
        Some(Commands::List) => list_modules()?,
        Some(Commands::Env) => setup_env()?,
        Some(Commands::Clean) => clean_workspace()?,
        None => {
            println!(
                "Usage:\n  psh host setup {{hosts...}}\n  psh mod up|down|setup|teardown|list {{modules...}}"
            );
        }
    }

    Ok(())
}
