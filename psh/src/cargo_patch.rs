use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use toml::{Value, map::Map};

use crate::workspace::{workspace_install, workspace_root};

#[derive(Debug, Clone)]
struct CratePatch {
    name: String,
    path: PathBuf,
}

pub fn refresh_cargo_patches() -> Result<()> {
    let workspace_root = workspace_root().context("failed to locate repository workspace root")?;
    let install_dir =
        workspace_install().context("failed to locate workspace install directory")?;
    let cargo_dir = workspace_root.join(".cargo");

    fs::create_dir_all(&cargo_dir).with_context(|| {
        format!(
            "failed to ensure Cargo config directory {} exists",
            cargo_dir.display()
        )
    })?;

    let patches = discover_rust_crate_patches(&install_dir)?;
    let config_path = cargo_dir.join("config.toml");

    write_patch_config(&config_path, &patches)?;

    Ok(())
}

fn discover_rust_crate_patches(install_dir: &Path) -> Result<Vec<CratePatch>> {
    if !install_dir.exists() {
        return Ok(Vec::new());
    }

    let mut crates = BTreeMap::<String, PathBuf>::new();

    for package_entry in fs::read_dir(install_dir).with_context(|| {
        format!(
            "failed to read workspace install directory {}",
            install_dir.display()
        )
    })? {
        let package_entry = package_entry?;
        let package_path = package_entry.path();
        let share_dir = package_path.join("share");
        if !share_dir.is_dir() {
            continue;
        }

        for interface_entry in fs::read_dir(&share_dir)
            .with_context(|| format!("failed to read share directory {}", share_dir.display()))?
        {
            let interface_entry = interface_entry?;
            let interface_path = interface_entry.path();
            let rust_dir = interface_path.join("rust");
            let manifest_path = rust_dir.join("Cargo.toml");
            if !manifest_path.is_file() {
                continue;
            }

            match collect_crate_patch(&manifest_path) {
                Ok((name, crate_path)) => {
                    crates.insert(name, crate_path);
                }
                Err(err) => {
                    eprintln!(
                        "[cargo-patch] Skipping {}: {}",
                        manifest_path.display(),
                        err
                    );
                }
            }
        }
    }

    Ok(crates
        .into_iter()
        .map(|(name, path)| CratePatch { name, path })
        .collect())
}

fn collect_crate_patch(manifest_path: &Path) -> Result<(String, PathBuf)> {
    let manifest = fs::read_to_string(manifest_path)
        .with_context(|| format!("failed to read Cargo manifest {}", manifest_path.display()))?;

    let parsed: Value = toml::from_str(&manifest)
        .with_context(|| format!("failed to parse Cargo manifest {}", manifest_path.display()))?;

    let package = parsed
        .get("package")
        .and_then(Value::as_table)
        .context("manifest missing [package] table")?;

    let name = package
        .get("name")
        .and_then(Value::as_str)
        .context("manifest missing package.name")?
        .to_string();

    let path = manifest_path
        .parent()
        .context("manifest missing parent directory")?
        .canonicalize()
        .with_context(|| {
            format!(
                "failed to canonicalize Rust crate directory {}",
                manifest_path.display()
            )
        })?;

    Ok((name, path))
}

fn write_patch_config(config_path: &Path, patches: &[CratePatch]) -> Result<()> {
    let existing = if config_path.is_file() {
        fs::read_to_string(config_path)
            .with_context(|| format!("failed to read {}", config_path.display()))?
    } else {
        String::new()
    };

    let mut root = if existing.trim().is_empty() {
        Value::Table(Map::new())
    } else {
        toml::from_str::<Value>(&existing).with_context(|| {
            format!(
                "failed to parse existing Cargo config {}",
                config_path.display()
            )
        })?
    };

    {
        let table = root
            .as_table_mut()
            .context("Cargo config is not a TOML table")?;

        let patch_value = table
            .entry("patch")
            .or_insert_with(|| Value::Table(Map::new()));

        let patch_table = patch_value
            .as_table_mut()
            .context("patch section is not a table")?;

        if patches.is_empty() {
            patch_table.remove("crates-io");
        } else {
            let mut crates_io = Map::new();
            for patch in patches {
                let mut entry = Map::new();
                entry.insert(
                    "path".to_string(),
                    Value::String(patch.path.to_string_lossy().into_owned()),
                );
                crates_io.insert(patch.name.clone(), Value::Table(entry));
            }
            patch_table.insert("crates-io".to_string(), Value::Table(crates_io));
        }

        if patch_table.is_empty() {
            table.remove("patch");
        }
    }

    let serialized = if matches!(root, Value::Table(ref t) if t.is_empty()) {
        String::new()
    } else {
        toml::to_string_pretty(&root).with_context(|| {
            format!(
                "failed to serialize updated Cargo config {}",
                config_path.display()
            )
        })?
    };

    fs::write(config_path, serialized).with_context(|| {
        format!(
            "failed to write updated Cargo config {}",
            config_path.display()
        )
    })?;

    Ok(())
}
