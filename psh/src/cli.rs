use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "psh", about = "psyched shell", version = env!("CARGO_PKG_VERSION"), propagate_version = true)]
pub struct Cli {
    #[command(subcommand)]
    pub command: Option<Commands>,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Host-related actions
    Host {
        #[command(subcommand)]
        command: HostCommands,
    },

    /// Module-related actions (use `psh mod ...`)
    Mod {
        #[command(subcommand)]
        command: ModCommands,
    },

    /// Build the ROS workspace (wraps `colcon build`)
    Build {
        /// Optional package names to limit the build (maps to `--packages-select`)
        #[arg(value_name = "PACKAGE")]
        packages: Vec<String>,
    },

    // Backwards-compatibility top-level aliases. These map to the same actions
    /// Run host bootstrap scripts (alias for `psh host setup`)
    #[command(hide = true)]
    Setup,

    /// Bring module(s) online (alias for `psh mod up`)
    #[command(hide = true)]
    Up {
        /// Module names (directories under modules/). When empty, defaults to all modules.
        #[arg(value_name = "MODULE")]
        modules: Vec<String>,
    },

    /// Gracefully stop module(s) (alias for `psh mod down`)
    #[command(hide = true)]
    Down {
        /// Module names (directories under modules/). When empty, defaults to all modules.
        #[arg(value_name = "MODULE")]
        modules: Vec<String>,
    },

    /// Prepare module assets (setup lifecycle) (alias for `psh mod setup`)
    #[command(name = "setup-module", hide = true)]
    SetupModule {
        /// Module name (directory under modules/)
        module: String,
    },

    /// Remove prepared module assets (teardown lifecycle) (alias for `psh mod teardown`)
    #[command(name = "teardown-module", hide = true)]
    TeardownModule {
        /// Module name (directory under modules/)
        module: String,
    },

    /// List available modules and their status (alias for `psh mod list`)
    #[command(hide = true)]
    List,

    /// Set up shell environment (add psyched alias to ~/.bashrc)
    Env,

    /// Remove build artifacts and recreate src symlinks for local crates
    Clean,
}

#[derive(Subcommand)]
pub enum HostCommands {
    /// Run host bootstrap for one or more hosts. If no hosts are provided the current hostname is used.
    Setup {
        /// Host names (matching files in hosts/*.toml). Defaults to the local hostname when empty.
        hosts: Vec<String>,
    },
}

#[derive(Subcommand)]
pub enum ModCommands {
    /// Bring module(s) online. If none provided, defaults to all modules.
    Up {
        /// Module names (directories under modules/)
        modules: Vec<String>,
    },
    /// Gracefully stop module(s). If none provided, defaults to all modules.
    Down {
        /// Module names (directories under modules/)
        modules: Vec<String>,
    },
    /// Prepare module assets (setup lifecycle). If none provided, defaults to all modules.
    Setup {
        /// Module names (directories under modules/)
        modules: Vec<String>,
    },
    /// Remove prepared module assets (teardown lifecycle). If none provided, defaults to all modules.
    Teardown {
        /// Module names (directories under modules/)
        modules: Vec<String>,
    },
    /// List available modules and their status
    List {
        /// Optional module names to filter the list
        modules: Vec<String>,
    },
}
