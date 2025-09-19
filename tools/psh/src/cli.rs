use clap::{Args, Parser, Subcommand, ValueHint};
use std::path::PathBuf;

#[derive(Parser, Debug)]
#[command(name = "psh", about = "Psyched shell utility", version)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Install the psh binary into the local PATH
    Install,
    /// Clone or update the canonical Psyched repository
    Clone,
    /// Build the psh utility from source
    Build,
    /// Clone, build, and install the utility in one go
    Update,
    /// Print environment sourcing instructions for the workspace
    Env,
    /// Provision ROS 2 using the bundled installer script
    Ros2(Ros2Args),
    /// Remove the installed utility and the cloned repository
    Remove,
    /// Manage host configurations
    Host {
        #[command(subcommand)]
        command: HostCommands,
    },
    /// Manage lifecycle operations for modules
    Module {
        #[command(subcommand)]
        command: ModuleCommands,
    },
}

#[derive(Subcommand, Debug)]
pub enum HostCommands {
    /// Apply the configuration for the given host
    Apply(HostApplyArgs),
}

#[derive(Args, Debug)]
pub struct HostApplyArgs {
    /// Hostname matching the TOML file under hosts/
    #[arg(value_hint = ValueHint::Other)]
    pub hostname: String,
}

#[derive(Subcommand, Debug)]
pub enum ModuleCommands {
    /// Run setup lifecycle for the provided modules
    Setup(ModuleArgs),
    /// Run teardown lifecycle for the provided modules
    Remove(ModuleArgs),
    /// Launch the provided modules as services
    Launch(ModuleArgs),
}

#[derive(Args, Debug)]
pub struct ModuleArgs {
    /// Names of the modules to operate on
    #[arg(required = true, value_hint = ValueHint::Other)]
    pub modules: Vec<String>,
}

#[derive(Args, Debug, Clone, Default)]
pub struct Ros2Args {
    /// ROS distribution to install (defaults to ROS_DISTRO env or kilted)
    #[arg(long, value_hint = ValueHint::Other)]
    pub distro: Option<String>,
    /// Optional override for the install script path
    #[arg(long, value_hint = ValueHint::FilePath)]
    pub script: Option<PathBuf>,
}
