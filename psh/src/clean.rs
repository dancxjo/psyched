use anyhow::{Context, Result};
use std::fs;
use std::os::unix::fs::symlink;
use std::path::Path;

use crate::setup::locate_repo_root;

pub fn clean_workspace() -> Result<()> {
    let repo_root = locate_repo_root().context("failed to locate repository root")?;

    println!("==> Removing ROS workspace artifacts");

    for entry in ["src", "build", "install"] {
        let target = repo_root.join(entry);
        remove_path(&target).with_context(|| format!("failed to remove {}", target.display()))?;
    }

    let src_dir = repo_root.join("src");
    fs::create_dir_all(&src_dir).context("failed to recreate src directory")?;

    // Keep the directory ignored in git while documenting its purpose for humans
    fs::write(src_dir.join(".gitignore"), "*\n!.gitignore\n")
        .context("failed to write src/.gitignore")?;

    println!("==> Re-establishing local package symlinks");
    for (name, target) in [
        ("psyched", "../psyched"),
        ("psyched-msgs", "../psyched-msgs"),
    ] {
        let link_path = src_dir.join(name);
        symlink(target, &link_path).with_context(|| {
            format!(
                "failed to create symlink {} -> {}",
                link_path.display(),
                target
            )
        })?;
    }

    println!("✓ Workspace cleaned");

    Ok(())
}

fn remove_path(path: &Path) -> Result<()> {
    match fs::symlink_metadata(path) {
        Ok(metadata) => {
            if metadata.file_type().is_symlink() || metadata.is_file() {
                fs::remove_file(path)
                    .with_context(|| format!("failed to remove file {}", path.display()))?;
            } else if metadata.is_dir() {
                fs::remove_dir_all(path)
                    .with_context(|| format!("failed to remove directory {}", path.display()))?;
            }
        }
        Err(err) if err.kind() == std::io::ErrorKind::NotFound => {
            // Nothing to remove
        }
        Err(err) => return Err(err.into()),
    }

    Ok(())
}
