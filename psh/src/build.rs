use anyhow::{Context, Result};
use duct::cmd;
use which::which;

use crate::setup::locate_repo_root;

pub fn build_workspace(packages: &[String]) -> Result<()> {
    which("colcon").with_context(|| {
        "colcon executable not found in PATH; install colcon before running `psh build`".to_string()
    })?;

    let repo_root = locate_repo_root().context("failed to locate psyched repository root")?;

    println!("==> Building ROS workspace at {}", repo_root.display());

    let mut args = vec!["build".to_string(), "--symlink-install".to_string()];

    if !packages.is_empty() {
        println!("   Selected packages: {}", packages.join(", "));
        args.push("--packages-select".to_string());
        args.extend(packages.iter().cloned());
    }

    cmd("colcon", args)
        .dir(&repo_root)
        .stderr_to_stdout()
        .run()
        .context("colcon build failed")?;

    println!("✓ Workspace build complete");

    Ok(())
}
