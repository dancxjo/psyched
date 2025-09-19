use anyhow::{bail, Context, Result};
use serde::Deserialize;
use std::fs;
use std::io;
use std::path::Path;

#[derive(Debug, Deserialize)]
pub struct HostConfig {
    pub name: String,
    #[serde(default)]
    pub modules: Vec<HostModule>,
}

impl HostConfig {
    pub fn load(dir: impl AsRef<Path>, hostname: &str) -> Result<Self> {
        let filename = format!("{hostname}.toml");
        let path = dir.as_ref().join(&filename);
        let contents = match fs::read_to_string(&path) {
            Ok(contents) => contents,
            Err(err) if err.kind() == io::ErrorKind::NotFound => {
                bail!("Unknown host: {hostname}")
            }
            Err(err) => {
                return Err(err).with_context(|| format!("failed to read {}", path.display()))
            }
        };
        let mut config: HostConfig = toml::from_str(&contents)
            .with_context(|| format!("failed to parse {}", path.display()))?;
        if config.name.is_empty() {
            config.name = hostname.to_string();
        }
        Ok(config)
    }
}

#[derive(Debug, Deserialize)]
pub struct HostModule {
    pub name: String,
    #[serde(default)]
    pub setup: bool,
    #[serde(default)]
    pub launch: bool,
    #[serde(default)]
    pub teardown: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::tempdir;

    #[test]
    fn host_config_parses_modules() {
        let toml = r#"
            name = "cerebellum"

            [[modules]]
            name = "foot"
            setup = true
            launch = true
        "#;
        let config: HostConfig = toml::from_str(toml).expect("valid config");
        assert_eq!(config.name, "cerebellum");
        assert_eq!(config.modules.len(), 1);
        let module = &config.modules[0];
        assert_eq!(module.name, "foot");
        assert!(module.setup);
        assert!(module.launch);
        assert!(!module.teardown);
    }

    #[test]
    fn load_reads_from_directory() {
        let dir = tempdir().unwrap();
        let file = dir.path().join("cerebellum.toml");
        fs::write(&file, "name = \"\"\n").unwrap();

        let config = HostConfig::load(dir.path(), "cerebellum").unwrap();
        assert_eq!(config.name, "cerebellum");
        assert!(config.modules.is_empty());
    }
}
