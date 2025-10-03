use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use toml::{Value, map::Map};

use crate::workspace::{repo_root, workspace_install, workspace_root};

#[derive(Debug, Clone)]
struct CratePatch {
    name: String,
    path: PathBuf,
}

pub fn refresh_cargo_patches() -> Result<()> {
    let workspace_root = workspace_root().context("failed to locate repository workspace root")?;
    let repo_root = repo_root().context("failed to locate repository root")?;
    let install_dir =
        workspace_install().context("failed to locate workspace install directory")?;

    let cargo_dir = workspace_root.join(".cargo");
    fs::create_dir_all(&cargo_dir).with_context(|| {
        format!(
            "failed to ensure Cargo config directory {} exists",
            cargo_dir.display()
        )
    })?;

    let mut crates = BTreeMap::<String, PathBuf>::new();

    let vendor_dir = repo_root.join("vendor_msgs");
    crates.extend(discover_vendor_crate_patches(&vendor_dir)?);

    let modules_dir = repo_root.join("modules");
    let module_crates = discover_module_crate_patches(&modules_dir, &install_dir)?;
    for (name, path) in module_crates {
        crates.insert(name, path);
    }

    let patches: Vec<CratePatch> = crates
        .into_iter()
        .map(|(name, path)| CratePatch { name, path })
        .collect();

    let config_path = cargo_dir.join("config.toml");
    write_patch_config(&config_path, &patches)?;

    Ok(())
}

fn discover_module_crate_patches(
    modules_dir: &Path,
    install_dir: &Path,
) -> Result<BTreeMap<String, PathBuf>> {
    if !modules_dir.is_dir() {
        return Ok(BTreeMap::new());
    }

    let mut crates = BTreeMap::<String, PathBuf>::new();

    for module_entry in fs::read_dir(modules_dir)
        .with_context(|| format!("failed to read modules directory {}", modules_dir.display()))?
    {
        let module_entry = module_entry?;
        let manifest_path = module_entry.path().join("module.toml");
        if !manifest_path.is_file() {
            continue;
        }

        let crate_names = collect_module_cargo_patches(&manifest_path)?;
        for crate_name in crate_names {
            if crates.contains_key(&crate_name) {
                continue;
            }

            match resolve_workspace_crate_dir(install_dir, &crate_name) {
                Ok(Some(crate_path)) => {
                    crates.insert(crate_name, crate_path);
                }
                Ok(None) => {
                    eprintln!(
                        "[cargo-patch] Skipping module patch '{}': crate not found under {}",
                        crate_name,
                        install_dir.display()
                    );
                }
                Err(err) => {
                    eprintln!(
                        "[cargo-patch] Skipping module patch '{}': {}",
                        crate_name, err
                    );
                }
            }
        }
    }

    Ok(crates)
}

fn collect_module_cargo_patches(manifest_path: &Path) -> Result<Vec<String>> {
    let manifest = fs::read_to_string(manifest_path)
        .with_context(|| format!("failed to read module manifest {}", manifest_path.display()))?;

    let parsed: Value = toml::from_str(&manifest).with_context(|| {
        format!(
            "failed to parse module manifest {}",
            manifest_path.display()
        )
    })?;

    let mut names = Vec::new();

    if let Some(units) = parsed.get("unit").and_then(Value::as_table) {
        for (unit_name, unit_value) in units {
            let Some(unit_table) = unit_value.as_table() else {
                continue;
            };

            if let Some(entries) = unit_table.get("cargo_patches").and_then(Value::as_array) {
                for entry in entries {
                    match entry.as_str() {
                        Some(name) => names.push(name.to_string()),
                        None => eprintln!(
                            "[cargo-patch] Ignoring non-string cargo_patches entry in {}::{}",
                            manifest_path.display(),
                            unit_name
                        ),
                    }
                }
            }
        }
    }

    names.sort();
    names.dedup();
    Ok(names)
}

fn resolve_workspace_crate_dir(install_dir: &Path, crate_name: &str) -> Result<Option<PathBuf>> {
    if !install_dir.exists() {
        return Ok(None);
    }

    let candidates = [
        install_dir
            .join(crate_name)
            .join("share")
            .join(crate_name)
            .join("rust"),
        install_dir.join("share").join(crate_name).join("rust"),
    ];

    for candidate in candidates {
        let manifest_path = candidate.join("Cargo.toml");
        if manifest_path.is_file() {
            let canonical = candidate.canonicalize().with_context(|| {
                format!(
                    "failed to canonicalize Rust crate directory {}",
                    candidate.display()
                )
            })?;
            return Ok(Some(canonical));
        }
    }

    Ok(None)
}

fn discover_vendor_crate_patches(vendor_dir: &Path) -> Result<BTreeMap<String, PathBuf>> {
    if !vendor_dir.is_dir() {
        return Ok(BTreeMap::new());
    }

    let mut crates = BTreeMap::<String, PathBuf>::new();

    for vendor_entry in fs::read_dir(vendor_dir).with_context(|| {
        format!(
            "failed to read vendor messages directory {}",
            vendor_dir.display()
        )
    })? {
        let vendor_entry = vendor_entry?;
        let manifest_path = vendor_entry.path().join("Cargo.toml");
        if !manifest_path.is_file() {
            continue;
        }

        match collect_crate_patch(&manifest_path) {
            Ok((name, crate_path)) => {
                crates.insert(name, crate_path);
            }
            Err(err) => {
                eprintln!(
                    "[cargo-patch] Skipping vendor crate manifest {}: {}",
                    manifest_path.display(),
                    err
                );
            }
        }
    }

    Ok(crates)
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
