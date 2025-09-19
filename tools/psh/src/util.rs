use anyhow::{bail, Context, Result};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};

pub fn ros_distro() -> String {
    env::var("ROS_DISTRO").unwrap_or_else(|_| "kilted".to_string())
}

pub fn ensure_tool(tool: &str, help: &str) -> Result<()> {
    if is_tool_available(tool) || allow_missing_dependencies() {
        return Ok(());
    }
    bail!("{tool} is required but was not found in PATH. {help}");
}

pub fn allow_missing_dependencies() -> bool {
    env::var("PSH_ALLOW_MISSING_DEPENDENCIES")
        .map(|v| v == "1")
        .unwrap_or(false)
}

pub fn is_tool_available(tool: &str) -> bool {
    env::var_os("PATH")
        .map(|paths| env::split_paths(&paths).any(|dir| is_executable(dir.join(tool))))
        .unwrap_or(false)
}

fn is_executable(path: PathBuf) -> bool {
    if !path.exists() {
        return false;
    }
    let metadata = match path.metadata() {
        Ok(m) => m,
        Err(_) => return false,
    };
    if !metadata.is_file() {
        return false;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mode = metadata.permissions().mode();
        mode & 0o111 != 0
    }
    #[cfg(not(unix))]
    {
        true
    }
}

pub fn set_executable_permissions(path: &Path) -> Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut permissions = fs::metadata(path)?.permissions();
        permissions.set_mode(0o755);
        fs::set_permissions(path, permissions)?;
    }
    Ok(())
}

pub fn sh_quote(value: &str) -> String {
    let escaped = value.replace('\'', "'\\''");
    format!("'{escaped}'")
}

pub fn psh_binary() -> Result<PathBuf> {
    if let Ok(path) = env::var("PSH_SELF") {
        return Ok(PathBuf::from(path));
    }
    env::current_exe().with_context(|| "unable to determine path to current psh executable")
}
