mod app;
mod cli;
mod command_runner;
mod host;
mod layout;
mod util;

use anyhow::Result;
use clap::Parser;

fn main() {
    if let Err(error) = run() {
        eprintln!("Error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let cli = cli::Cli::parse();
    let app = app::App::default()?;
    app.execute(cli.command)
}
