mod cli;
mod module_runner;
mod setup;

use anyhow::Result;
use clap::Parser;

use crate::cli::{Cli, Commands};
use crate::module_runner::{
    bring_module_down, bring_module_up, list_modules, setup_module, teardown_module,
};
use crate::setup::run_setup;

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Some(Commands::Setup) => run_setup()?,
        Some(Commands::Up { module }) => bring_module_up(&module)?,
        Some(Commands::Down { module }) => bring_module_down(&module)?,
        Some(Commands::SetupModule { module }) => setup_module(&module)?,
        Some(Commands::TeardownModule { module }) => teardown_module(&module)?,
        Some(Commands::List) => list_modules()?,
        None => {
            println!("Usage:\n  psh setup\n  psh up <module>\n  psh down <module>\n  psh list");
        }
    }

    Ok(())
}
