use anyhow::{Context, Result};
use duct::cmd;
use which::which;

use crate::cargo_patch::refresh_cargo_patches;
use crate::workspace::workspace_root;

pub fn build_workspace(packages: &[String]) -> Result<()> {
    which("colcon").with_context(|| {
        "colcon executable not found in PATH; install colcon before running `psh build`".to_string()
    })?;

    let workspace_root = workspace_root().context("failed to locate ROS workspace root")?;

    println!("==> Building ROS workspace at {}", workspace_root.display());

    refresh_cargo_patches()?;

    let mut args = vec!["build".to_string(), "--symlink-install".to_string()];

    if !packages.is_empty() {
        println!("   Selected packages: {}", packages.join(", "));
        args.push("--packages-select".to_string());
        args.extend(packages.iter().cloned());
    }

    cmd("colcon", args)
        .dir(&workspace_root)
        .stderr_to_stdout()
        .run()
        .context("colcon build failed")?;

    refresh_cargo_patches()?;

    println!("✓ Workspace build complete");

    Ok(())
}
