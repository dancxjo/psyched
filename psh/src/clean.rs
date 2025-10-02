use anyhow::{Context, Result, bail};
use std::process::Command;

use crate::workspace::repo_root;

pub fn clean_workspace() -> Result<()> {
    let repo_root = repo_root().context("failed to locate repository root")?;
    let tool = repo_root.join("tools/clean_workspace");

    if !tool.exists() {
        bail!("workspace clean tool not found at {}", tool.display());
    }

    let status = Command::new(&tool)
        .current_dir(&repo_root)
        .status()
        .with_context(|| format!("failed to execute {}", tool.display()))?;

    if !status.success() {
        bail!(
            "workspace clean tool exited with status {:?}",
            status.code()
        );
    }

    Ok(())
}
