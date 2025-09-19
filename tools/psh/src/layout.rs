use anyhow::{Context, Result};
use std::env;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const DEFAULT_REPO_PATH: &str = "/opt/psyched";
const DEFAULT_SYSTEMD_DIR: &str = "/etc/systemd/system";
const DEFAULT_SYSTEM_INSTALL_DIR: &str = "/usr/bin";
const DEFAULT_INSTALL_SUBDIR: &str = ".local/bin";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Layout {
    pub repo_path: PathBuf,
    pub install_dir: PathBuf,
    pub install_dir_overridden: bool,
    pub systemd_dir: PathBuf,
}

impl Layout {
    pub fn detect() -> Result<Self> {
        let repo_path = env::var("PSH_REPO_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from(DEFAULT_REPO_PATH));

        let (install_dir, install_dir_overridden) = match env::var("PSH_INSTALL_DIR") {
            Ok(dir) => (PathBuf::from(dir), true),
            Err(_) => (default_install_dir(), false),
        };

        let systemd_dir = env::var("PSH_SYSTEMD_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from(DEFAULT_SYSTEMD_DIR));

        Ok(Self {
            repo_path,
            install_dir,
            install_dir_overridden,
            systemd_dir,
        })
    }

    #[cfg(test)]
    pub fn new(repo_path: PathBuf, install_dir: PathBuf, systemd_dir: PathBuf) -> Self {
        Self::with_install_dir_source(repo_path, install_dir, systemd_dir, true)
    }

    #[cfg(test)]
    pub fn with_install_dir_source(
        repo_path: PathBuf,
        install_dir: PathBuf,
        systemd_dir: PathBuf,
        install_dir_overridden: bool,
    ) -> Self {
        Self {
            repo_path,
            install_dir,
            install_dir_overridden,
            systemd_dir,
        }
    }

    pub fn release_binary(&self) -> PathBuf {
        self.repo_path.join("target").join("release").join("psh")
    }

    pub fn installed_binary(&self) -> PathBuf {
        self.install_dir.join("psh")
    }

    pub fn ensure_install_dir(&self) -> Result<()> {
        if !self.install_dir.exists() {
            fs::create_dir_all(&self.install_dir)
                .with_context(|| format!("unable to create {}", self.install_dir.display()))?;
        }
        Ok(())
    }

    pub fn ros2_install_script(&self) -> PathBuf {
        self.repo_path.join("tools").join("install_ros2.sh")
    }

    pub fn setup_env_script(&self) -> PathBuf {
        self.repo_path.join("tools").join("setup_env.sh")
    }

    pub fn hosts_dir(&self) -> PathBuf {
        self.repo_path.join("hosts")
    }

    pub fn modules_dir(&self) -> PathBuf {
        self.repo_path.join("modules")
    }
}

pub fn default_install_dir() -> PathBuf {
    PathBuf::from(DEFAULT_SYSTEM_INSTALL_DIR)
}

pub fn user_install_dir() -> PathBuf {
    let home = env::var("HOME").unwrap_or_else(|_| String::from("/root"));
    PathBuf::from(home).join(DEFAULT_INSTALL_SUBDIR)
}

pub fn copy_release_binary(release: &Path, target: &Path) -> io::Result<()> {
    if let Some(dir) = target.parent() {
        if !dir.exists() {
            fs::create_dir_all(dir)?;
        }
    }
    fs::copy(release, target).map(|_| ())
}

#[cfg(test)]
impl Layout {
    pub fn default_install_subdir_for_tests() -> &'static str {
        DEFAULT_INSTALL_SUBDIR
    }
}
