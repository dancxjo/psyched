use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "psh", about = "Pete's setup helper")]
pub struct Cli {
    #[command(subcommand)]
    pub command: Option<Commands>,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Run host bootstrap scripts
    Setup,
    /// Bring a module online
    Up {
        /// Module name (directory under modules/)
        module: String,
    },
    /// Gracefully stop a module
    Down {
        /// Module name (directory under modules/)
        module: String,
    },
    /// Prepare module assets (setup lifecycle)
    #[command(name = "setup-module")]
    SetupModule {
        /// Module name (directory under modules/)
        module: String,
    },
    /// Remove prepared module assets (teardown lifecycle)
    #[command(name = "teardown-module")]
    TeardownModule {
        /// Module name (directory under modules/)
        module: String,
    },
    /// List available modules and their status
    List,
    /// Set up shell environment (add psyched alias to ~/.bashrc)
    Env,
}
