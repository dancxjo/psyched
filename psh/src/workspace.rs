use anyhow::{Result, bail};
use directories::BaseDirs;
use std::env;
use std::path::{Path, PathBuf};

fn is_repo_root(candidate: &Path) -> bool {
    candidate.join("modules").is_dir() && candidate.join("psh").join("Cargo.toml").is_file()
}

fn expand_tilde(path: &str) -> PathBuf {
    if let Some(stripped) = path.strip_prefix("~/") {
        if let Some(base_dirs) = BaseDirs::new() {
            return base_dirs.home_dir().join(stripped);
        }
    }

    PathBuf::from(path)
}

fn resolve_within_workspace(var: &str, default: &str) -> Result<PathBuf> {
    if let Ok(dir) = env::var(var) {
        let expanded = expand_tilde(&dir);
        if expanded.is_absolute() {
            return Ok(expanded);
        } else {
            return Ok(workspace_root()?.join(expanded));
        }
    }

    Ok(workspace_root()?.join(default))
}

pub fn repo_root() -> Result<PathBuf> {
    if let Ok(root) = env::var("PSYCHED_REPO_ROOT") {
        let candidate = PathBuf::from(root);
        if is_repo_root(&candidate) {
            return Ok(candidate);
        }
    }

    let mut candidates = Vec::new();

    if let Ok(cwd) = env::current_dir() {
        candidates.push(cwd);
    }

    if let Ok(exe) = env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.push(dir.to_path_buf());
            if let Some(parent) = dir.parent() {
                candidates.push(parent.to_path_buf());
            }
        }
    }

    for candidate in candidates {
        if is_repo_root(&candidate) {
            return Ok(candidate);
        }

        let mut current = candidate.as_path();
        while let Some(dir) = current.parent() {
            if is_repo_root(dir) {
                return Ok(dir.to_path_buf());
            }
            current = dir;
        }
    }

    bail!("could not determine psyched repository root; run 'psh env' from inside the repository")
}

pub fn workspace_root() -> Result<PathBuf> {
    if let Ok(dir) = env::var("PSYCHED_WORKSPACE_DIR") {
        let expanded = expand_tilde(&dir);
        if expanded.is_absolute() {
            return Ok(expanded);
        } else {
            return Ok(repo_root()?.join(expanded));
        }
    }

    Ok(repo_root()?.join("work"))
}

pub fn workspace_src() -> Result<PathBuf> {
    resolve_within_workspace("PSYCHED_WORKSPACE_SRC", "src")
}

pub fn workspace_install() -> Result<PathBuf> {
    resolve_within_workspace("PSYCHED_WORKSPACE_INSTALL", "install")
}
