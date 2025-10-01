mod cli;
mod setup;

use anyhow::Result;
use clap::Parser;

use crate::cli::{Cli, Commands};
use crate::setup::run_setup;

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::Setup) => run_setup()?,
        None => {
            println!("Usage:\n  psh setup");
        }
    }

    Ok(())
}
