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
}
