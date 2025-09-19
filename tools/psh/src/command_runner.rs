use anyhow::{bail, Context, Result};
use std::path::PathBuf;
use std::process::Command;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CommandSpec {
    pub program: String,
    pub args: Vec<String>,
    pub cwd: Option<PathBuf>,
}

impl CommandSpec {
    pub fn new(program: impl Into<String>) -> Self {
        Self {
            program: program.into(),
            args: Vec::new(),
            cwd: None,
        }
    }

    pub fn arg(mut self, arg: impl Into<String>) -> Self {
        self.args.push(arg.into());
        self
    }

    #[allow(dead_code)]
    pub fn args<I, S>(mut self, args: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        self.args.extend(args.into_iter().map(Into::into));
        self
    }

    pub fn cwd(mut self, path: impl Into<PathBuf>) -> Self {
        self.cwd = Some(path.into());
        self
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct CommandOutput {
    pub status: i32,
}

impl CommandOutput {
    #[cfg(test)]
    pub fn success() -> Self {
        Self { status: 0 }
    }
}

pub trait CommandRunner {
    fn run(&self, spec: &CommandSpec) -> Result<CommandOutput>;
}

#[derive(Clone, Debug, Default)]
pub struct RealCommandRunner;

impl CommandRunner for RealCommandRunner {
    fn run(&self, spec: &CommandSpec) -> Result<CommandOutput> {
        let mut command = Command::new(&spec.program);
        command.args(&spec.args);
        if let Some(cwd) = &spec.cwd {
            command.current_dir(cwd);
        }

        let display = format_command(spec);
        println!("→ {}", display);

        let status = command
            .status()
            .with_context(|| format!("failed to spawn {display}"))?;
        if !status.success() {
            bail!("{} failed with status {}", display, status);
        }
        Ok(CommandOutput {
            status: status.code().unwrap_or_default(),
        })
    }
}

pub fn format_command(spec: &CommandSpec) -> String {
    let mut parts = vec![spec.program.clone()];
    parts.extend(spec.args.clone());
    if let Some(cwd) = &spec.cwd {
        parts.push(format!("(cwd: {})", cwd.display()));
    }
    parts.join(" ")
}
