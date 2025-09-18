use anyhow::{anyhow, bail, Context, Result};
use clap::{Args, Parser, Subcommand, ValueHint};
use include_dir::{include_dir, Dir};
use serde::Deserialize;
use std::env;
use std::ffi::OsString;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};
use std::process::Command;

const REPO_URL: &str = "https://github.com/dancxjo/psyched.git";
const DEFAULT_REPO_PATH: &str = "$HOME/psyched";
const DEFAULT_SYSTEM_INSTALL_DIR: &str = "/usr/bin";
const DEFAULT_INSTALL_SUBDIR: &str = ".local/bin";
const DEFAULT_SYSTEMD_DIR: &str = "/etc/systemd/system";
const FOOT_SERVICE_NAME: &str = "psyched-foot.service";
const CREATE_ROBOT_REPO: &str = "https://github.com/autonomylab/create_robot.git";
const LIBCREATE_REPO: &str = "https://github.com/revyos-ros/libcreate.git";
const LIBCREATE_BRANCH: &str = "fix-std-string";

static HOSTS_DIR: Dir<'_> = include_dir!("$CARGO_MANIFEST_DIR/hosts");

#[derive(Parser, Debug)]
#[command(name = "psh", about = "Psyched shell utility", version)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Install the psh binary into the local PATH
    Install,
    /// Clone or update the canonical Psyched repository
    Clone,
    /// Build the psh utility from source
    Build,
    /// Clone, build, and install the utility in one go
    Update,
    /// Provision ROS 2 using the bundled installer script
    Ros2(Ros2Args),
    /// Remove the installed utility and the cloned repository
    Remove,
    /// Manage host configurations
    Host {
        #[command(subcommand)]
        command: HostCommands,
    },
    /// Manage lifecycle operations for modules
    Module {
        #[command(subcommand)]
        command: ModuleCommands,
    },
}

#[derive(Subcommand, Debug)]
enum HostCommands {
    /// Apply the configuration for the given host
    Apply(HostApplyArgs),
}

#[derive(Args, Debug)]
struct HostApplyArgs {
    /// Hostname matching the TOML file under hosts/
    #[arg(value_hint = ValueHint::Other)]
    hostname: String,
}

#[derive(Subcommand, Debug)]
enum ModuleCommands {
    /// Run setup lifecycle for the provided modules
    Setup(ModuleArgs),
    /// Run teardown lifecycle for the provided modules
    Remove(ModuleArgs),
    /// Launch the provided modules as services
    Launch(ModuleArgs),
}

#[derive(Args, Debug)]
struct ModuleArgs {
    /// Names of the modules to operate on
    #[arg(required = true, value_hint = ValueHint::Other)]
    modules: Vec<String>,
}

#[derive(Args, Debug, Clone, Default)]
struct Ros2Args {
    /// ROS distribution to install (defaults to ROS_DISTRO env or kilted)
    #[arg(long, value_hint = ValueHint::Other)]
    distro: Option<String>,
    /// Optional override for the install script path
    #[arg(long, value_hint = ValueHint::FilePath)]
    script: Option<PathBuf>,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct CommandSpec {
    program: String,
    args: Vec<String>,
    cwd: Option<PathBuf>,
}

impl CommandSpec {
    fn new(program: impl Into<String>) -> Self {
        Self {
            program: program.into(),
            args: Vec::new(),
            cwd: None,
        }
    }

    fn arg(mut self, arg: impl Into<String>) -> Self {
        self.args.push(arg.into());
        self
    }

    fn args<I, S>(mut self, args: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        self.args.extend(args.into_iter().map(Into::into));
        self
    }

    fn cwd(mut self, path: impl Into<PathBuf>) -> Self {
        self.cwd = Some(path.into());
        self
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct CommandOutput {
    status: i32,
}

impl CommandOutput {
    #[cfg(test)]
    fn success() -> Self {
        Self { status: 0 }
    }
}

trait CommandRunner {
    fn run(&self, spec: &CommandSpec) -> Result<CommandOutput>;
}

#[derive(Clone, Debug, Default)]
struct RealCommandRunner;

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

fn format_command(spec: &CommandSpec) -> String {
    let mut parts = vec![spec.program.clone()];
    parts.extend(spec.args.clone());
    if let Some(cwd) = &spec.cwd {
        parts.push(format!("(cwd: {})", cwd.display()));
    }
    parts.join(" ")
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Layout {
    repo_path: PathBuf,
    install_dir: PathBuf,
    install_dir_overridden: bool,
    systemd_dir: PathBuf,
}

impl Layout {
    fn detect() -> Result<Self> {
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
    fn new(repo_path: PathBuf, install_dir: PathBuf, systemd_dir: PathBuf) -> Self {
        Self::with_install_dir_source(repo_path, install_dir, systemd_dir, true)
    }

    #[cfg(test)]
    fn with_install_dir_source(
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

    fn release_binary(&self) -> PathBuf {
        self.repo_path.join("target").join("release").join("psh")
    }

    fn installed_binary(&self) -> PathBuf {
        self.install_dir.join("psh")
    }

    fn ensure_install_dir(&self) -> Result<()> {
        if !self.install_dir.exists() {
            fs::create_dir_all(&self.install_dir)
                .with_context(|| format!("unable to create {}", self.install_dir.display()))?;
        }
        Ok(())
    }

    fn service_path(&self, filename: &str) -> PathBuf {
        self.systemd_dir.join(filename)
    }

    fn ros2_install_script(&self) -> PathBuf {
        self.repo_path.join("scripts").join("install_ros2.sh")
    }
}

fn default_install_dir() -> PathBuf {
    PathBuf::from(DEFAULT_SYSTEM_INSTALL_DIR)
}

fn user_install_dir() -> PathBuf {
    let home = env::var("HOME").unwrap_or_else(|_| String::from("/root"));
    PathBuf::from(home).join(DEFAULT_INSTALL_SUBDIR)
}

fn copy_release_binary(release: &Path, target: &Path) -> io::Result<()> {
    if let Some(dir) = target.parent() {
        if !dir.exists() {
            fs::create_dir_all(dir)?;
        }
    }
    fs::copy(release, target).map(|_| ())
}

#[derive(Clone, Debug)]
struct Psh<R: CommandRunner> {
    layout: Layout,
    runner: R,
}

impl<R: CommandRunner> Psh<R> {
    fn new(layout: Layout, runner: R) -> Self {
        Self { layout, runner }
    }

    fn install(&self) -> Result<()> {
        let release = self.layout.release_binary();
        if !release.exists() {
            bail!(
                "{} was not found. Run `psh build` first.",
                release.display()
            );
        }
        self.layout.ensure_install_dir()?;
        let primary_target = self.layout.installed_binary();
        match copy_release_binary(&release, &primary_target) {
            Ok(()) => {
                set_executable_permissions(&primary_target)?;
                println!(
                    "Installed {} to {}",
                    release.display(),
                    primary_target.display()
                );
                Ok(())
            }
            Err(err)
                if err.kind() == io::ErrorKind::PermissionDenied
                    && !self.layout.install_dir_overridden =>
            {
                let fallback_dir = user_install_dir();
                let fallback_target = fallback_dir.join("psh");
                copy_release_binary(&release, &fallback_target).with_context(|| {
                    format!(
                        "failed to copy {} to {}",
                        release.display(),
                        fallback_target.display()
                    )
                })?;
                set_executable_permissions(&fallback_target)?;
                println!(
                    "Insufficient permissions for {}. Installed {} instead.",
                    primary_target.display(),
                    fallback_target.display()
                );
                Ok(())
            }
            Err(err) => Err::<(), io::Error>(err).with_context(|| {
                format!(
                    "failed to copy {} to {}",
                    release.display(),
                    primary_target.display()
                )
            }),
        }
    }

    fn clone_repo(&self) -> Result<()> {
        ensure_tool(
            "git",
            "Install Git by following https://git-scm.com/downloads",
        )?;
        if self.layout.repo_path.exists() {
            if self.layout.repo_path.join(".git").exists() {
                self.runner.run(
                    &CommandSpec::new("git")
                        .arg("-C")
                        .arg(self.layout.repo_path.to_string_lossy())
                        .arg("pull")
                        .arg("--ff-only"),
                )?;
            } else {
                bail!(
                    "{} already exists but is not a git repository. Remove it or point PSH_REPO_DIR elsewhere.",
                    self.layout.repo_path.display()
                );
            }
        } else {
            if let Some(parent) = self.layout.repo_path.parent() {
                if !parent.exists() {
                    fs::create_dir_all(parent)
                        .with_context(|| format!("unable to create {}", parent.display()))?;
                }
            }
            self.runner.run(
                &CommandSpec::new("git")
                    .arg("clone")
                    .arg(REPO_URL)
                    .arg(self.layout.repo_path.to_string_lossy()),
            )?;
        }
        Ok(())
    }

    fn build(&self) -> Result<()> {
        ensure_tool("cargo", "Install Rust toolchain via https://rustup.rs")?;
        self.runner.run(
            &CommandSpec::new("cargo")
                .arg("build")
                .arg("--release")
                .cwd(&self.layout.repo_path),
        )?;
        Ok(())
    }

    fn update(&self) -> Result<()> {
        self.clone_repo()?;
        self.build()?;
        self.install()?;
        Ok(())
    }

    fn install_ros2(&self, args: &Ros2Args) -> Result<()> {
        ensure_tool(
            "bash",
            "Install bash so the ROS 2 provisioning script can be executed.",
        )?;

        let script_path = args
            .script
            .clone()
            .unwrap_or_else(|| self.layout.ros2_install_script());

        if !script_path.exists() {
            bail!(
                "{} does not exist. Run `psh clone` first or provide --script to point to install_ros2.sh.",
                script_path.display()
            );
        }

        let distro = args.distro.clone().unwrap_or_else(ros_distro);

        let original: Option<OsString> = env::var_os("ROS_DISTRO");
        env::set_var("ROS_DISTRO", &distro);

        let result = self
            .runner
            .run(&CommandSpec::new("bash").arg(script_path.to_string_lossy()));

        match original {
            Some(value) => env::set_var("ROS_DISTRO", value),
            None => env::remove_var("ROS_DISTRO"),
        }

        result?;
        println!(
            "Invoked ROS 2 {distro} provisioning via {}",
            script_path.display()
        );
        Ok(())
    }

    fn remove(&self) -> Result<()> {
        let target = self.layout.installed_binary();
        if target.exists() {
            fs::remove_file(&target)
                .with_context(|| format!("unable to remove {}", target.display()))?;
            println!("Removed {}", target.display());
        }
        if self.layout.repo_path.exists() {
            fs::remove_dir_all(&self.layout.repo_path)
                .with_context(|| format!("unable to remove {}", self.layout.repo_path.display()))?;
            println!("Removed {}", self.layout.repo_path.display());
        }
        Ok(())
    }

    fn module_action(&self, name: &str, action: ModuleAction) -> Result<()> {
        match (name, action) {
            ("foot", ModuleAction::Setup) => self.foot_setup(),
            ("foot", ModuleAction::Remove) => self.foot_teardown(),
            ("foot", ModuleAction::Launch) => self.foot_launch(),
            _ => bail!("Unknown module: {name}"),
        }
    }

    fn host_apply(&self, hostname: &str) -> Result<()> {
        let config = HostConfig::load(hostname)?;
        for module in &config.modules {
            if module.setup {
                self.module_action(&module.name, ModuleAction::Setup)?;
            }
            if module.launch {
                self.module_action(&module.name, ModuleAction::Launch)?;
            }
            if module.teardown {
                self.module_action(&module.name, ModuleAction::Remove)?;
            }
        }
        Ok(())
    }

    fn foot_setup(&self) -> Result<()> {
        ensure_tool(
            "git",
            "Install Git by following https://git-scm.com/downloads",
        )?;
        ensure_tool(
            "rosdep",
            "Install rosdep (https://docs.ros.org) before continuing.",
        )?;
        ensure_tool(
            "colcon",
            "Install colcon by following https://colcon.readthedocs.io",
        )?;

        if !self.layout.repo_path.exists() {
            bail!(
                "{} does not exist. Run `psh clone` first or set PSH_REPO_DIR to an existing checkout.",
                self.layout.repo_path.display()
            );
        }

        let source_dir = self.layout.repo_path.join("src");
        fs::create_dir_all(&source_dir)
            .with_context(|| format!("unable to create {}", source_dir.display()))?;

        let create_robot_path = source_dir.join("create_robot");
        if !create_robot_path.join(".git").exists() {
            self.runner.run(
                &CommandSpec::new("git")
                    .arg("clone")
                    .arg(CREATE_ROBOT_REPO)
                    .arg(create_robot_path.to_string_lossy()),
            )?;
        }

        let libcreate_path = source_dir.join("libcreate");
        if !libcreate_path.join(".git").exists() {
            self.runner.run(
                &CommandSpec::new("git")
                    .arg("clone")
                    .arg("--branch")
                    .arg(LIBCREATE_BRANCH)
                    .arg("--single-branch")
                    .arg(LIBCREATE_REPO)
                    .arg(libcreate_path.to_string_lossy()),
            )?;
        }

        let distro = ros_distro();
        let script = format!(
            "set -euo pipefail\nsource /opt/ros/{distro}/setup.bash\ncd {repo}\nrosdep install -i --from-path src --rosdistro {distro} -y\ncolcon build --symlink-install\n",
            repo = self.layout.repo_path.display()
        );
        self.runner.run(
            &CommandSpec::new("bash")
                .arg("-lc")
                .arg(script)
                .cwd(&self.layout.repo_path),
        )?;
        Ok(())
    }

    fn foot_teardown(&self) -> Result<()> {
        let source_dir = self.layout.repo_path.join("src");
        for dir in ["create_robot", "libcreate"] {
            let path = source_dir.join(dir);
            if path.exists() {
                fs::remove_dir_all(&path)
                    .with_context(|| format!("unable to remove {}", path.display()))?;
                println!("Removed {}", path.display());
            }
        }

        if is_tool_available("systemctl") {
            let service = FOOT_SERVICE_NAME;
            let _ = self
                .runner
                .run(&CommandSpec::new("systemctl").args(["disable", "--now", service]));
        }

        let service_path = self.layout.service_path(FOOT_SERVICE_NAME);
        if service_path.exists() {
            fs::remove_file(&service_path)
                .with_context(|| format!("unable to remove {}", service_path.display()))?;
            println!("Removed {}", service_path.display());
        }
        Ok(())
    }

    fn foot_launch(&self) -> Result<()> {
        ensure_tool(
            "systemctl",
            "Install systemd or run on a systemd-enabled host.",
        )?;
        ensure_tool(
            "bash",
            "A POSIX shell is required to source ROS environments.",
        )?;

        let distro = ros_distro();
        let service_path = self.layout.service_path(FOOT_SERVICE_NAME);
        if let Some(parent) = service_path.parent() {
            fs::create_dir_all(parent)
                .with_context(|| format!("unable to create {}", parent.display()))?;
        }

        let user = default_service_user();
        let service_contents = format!(
            "[Unit]\nDescription=Psyched foot module bringup\nAfter=network.target\n\n[Service]\nType=simple\nUser={user}\nEnvironment=ROS_DOMAIN_ID=0\nExecStart=/bin/bash -lc 'source /opt/ros/{distro}/setup.bash && cd {repo} && source install/setup.bash && ros2 launch create_bringup create_1.launch'\nRestart=on-failure\n\n[Install]\nWantedBy=multi-user.target\n",
            repo = self.layout.repo_path.display()
        );
        fs::write(&service_path, service_contents)
            .with_context(|| format!("unable to write {}", service_path.display()))?;

        self.runner
            .run(&CommandSpec::new("systemctl").args(["daemon-reload"]))?;
        self.runner.run(&CommandSpec::new("systemctl").args([
            "enable",
            "--now",
            FOOT_SERVICE_NAME,
        ]))?;
        println!(
            "Foot module service installed at {} and enabled via systemd.",
            service_path.display()
        );
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
enum ModuleAction {
    Setup,
    Remove,
    Launch,
}

#[derive(Debug, Deserialize)]
struct HostConfig {
    name: String,
    #[serde(default)]
    modules: Vec<HostModule>,
}

impl HostConfig {
    fn load(hostname: &str) -> Result<Self> {
        let filename = format!("{hostname}.toml");
        let file = HOSTS_DIR
            .get_file(&filename)
            .ok_or_else(|| anyhow!("Unknown host: {hostname}"))?;
        let contents = file
            .contents_utf8()
            .ok_or_else(|| anyhow!("{filename} is not valid UTF-8"))?;
        let mut config: HostConfig = toml::from_str(contents)
            .with_context(|| format!("failed to parse hosts/{filename}"))?;
        if config.name.is_empty() {
            config.name = hostname.to_string();
        }
        Ok(config)
    }
}

#[derive(Debug, Deserialize)]
struct HostModule {
    name: String,
    #[serde(default)]
    setup: bool,
    #[serde(default)]
    launch: bool,
    #[serde(default)]
    teardown: bool,
}

fn ros_distro() -> String {
    env::var("ROS_DISTRO").unwrap_or_else(|_| "kilted".to_string())
}

fn default_service_user() -> String {
    env::var("PSH_SERVICE_USER")
        .or_else(|_| env::var("USER"))
        .unwrap_or_else(|_| "root".to_string())
}

fn is_tool_available(tool: &str) -> bool {
    env::var_os("PATH")
        .map(|paths| env::split_paths(&paths).any(|dir| is_executable(dir.join(tool))))
        .unwrap_or(false)
}

fn ensure_tool(tool: &str, help: &str) -> Result<()> {
    if is_tool_available(tool) || allow_missing_dependencies() {
        return Ok(());
    }
    bail!("{tool} is required but was not found in PATH. {help}");
}

fn allow_missing_dependencies() -> bool {
    env::var("PSH_ALLOW_MISSING_DEPENDENCIES")
        .map(|v| v == "1")
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

fn set_executable_permissions(path: &Path) -> Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let mut permissions = fs::metadata(path)?.permissions();
        permissions.set_mode(0o755);
        fs::set_permissions(path, permissions)?;
    }
    Ok(())
}

fn main() {
    if let Err(error) = run() {
        eprintln!("Error: {error}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let layout = Layout::detect()?;
    let psh = Psh::new(layout, RealCommandRunner);

    match cli.command {
        Commands::Install => psh.install(),
        Commands::Clone => psh.clone_repo(),
        Commands::Build => psh.build(),
        Commands::Update => psh.update(),
        Commands::Ros2(args) => psh.install_ros2(&args),
        Commands::Remove => psh.remove(),
        Commands::Host { command } => match command {
            HostCommands::Apply(args) => psh.host_apply(&args.hostname),
        },
        Commands::Module { command } => match command {
            ModuleCommands::Setup(args) => {
                for module in &args.modules {
                    psh.module_action(module, ModuleAction::Setup)?;
                }
                Ok(())
            }
            ModuleCommands::Remove(args) => {
                for module in &args.modules {
                    psh.module_action(module, ModuleAction::Remove)?;
                }
                Ok(())
            }
            ModuleCommands::Launch(args) => {
                for module in &args.modules {
                    psh.module_action(module, ModuleAction::Launch)?;
                }
                Ok(())
            }
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;
    use std::sync::{Mutex, OnceLock};
    use tempfile::tempdir;

    fn env_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    #[derive(Clone, Default)]
    struct MockRunner {
        executed: Rc<RefCell<Vec<CommandSpec>>>,
    }

    impl CommandRunner for MockRunner {
        fn run(&self, spec: &CommandSpec) -> Result<CommandOutput> {
            self.executed.borrow_mut().push(spec.clone());
            Ok(CommandOutput::success())
        }
    }

    impl MockRunner {
        fn commands(&self) -> Vec<CommandSpec> {
            self.executed.borrow().clone()
        }
    }

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
    fn install_copies_release_binary() {
        let repo_dir = tempdir().unwrap();
        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );
        let runner = MockRunner::default();
        let psh = Psh::new(layout.clone(), runner);

        let release = layout.release_binary();
        fs::create_dir_all(release.parent().unwrap()).unwrap();
        fs::write(&release, b"fake binary").unwrap();

        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");
        psh.install().expect("install succeeds");

        let installed = layout.installed_binary();
        assert!(installed.exists());
        let contents = fs::read(installed).unwrap();
        assert_eq!(contents, b"fake binary");
        env::remove_var("PSH_ALLOW_MISSING_DEPENDENCIES");
    }

    #[cfg(unix)]
    #[test]
    fn install_without_permissions_falls_back_to_user_dir() {
        use std::os::unix::fs::PermissionsExt;

        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");

        let repo_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let restricted_root = tempdir().unwrap();
        let primary_dir = restricted_root.path().join("bin");
        fs::create_dir_all(&primary_dir).unwrap();
        fs::set_permissions(&primary_dir, fs::Permissions::from_mode(0o555)).unwrap();

        let layout = Layout::with_install_dir_source(
            repo_dir.path().into(),
            primary_dir.clone(),
            systemd_dir.path().into(),
            false,
        );

        let runner = MockRunner::default();
        let psh = Psh::new(layout.clone(), runner);

        let release = layout.release_binary();
        fs::create_dir_all(release.parent().unwrap()).unwrap();
        fs::write(&release, b"fallback binary").unwrap();

        let home_dir = tempdir().unwrap();
        let original_home = env::var("HOME").ok();
        env::set_var("HOME", home_dir.path());

        let result = psh.install();
        assert!(result.is_ok(), "install should succeed with fallback");

        let fallback_path = home_dir
            .path()
            .join(DEFAULT_INSTALL_SUBDIR)
            .join("psh");
        assert!(fallback_path.exists(), "fallback binary should be installed");
        assert!(!primary_dir.join("psh").exists(), "primary path should remain untouched");
        let contents = fs::read(fallback_path).unwrap();
        assert_eq!(contents, b"fallback binary");

        if let Some(home) = original_home {
            env::set_var("HOME", home);
        } else {
            env::remove_var("HOME");
        }
        env::remove_var("PSH_ALLOW_MISSING_DEPENDENCIES");
    }

    #[test]
    fn foot_setup_clones_repositories() {
        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");
        let repo_dir = tempdir().unwrap();
        fs::create_dir_all(repo_dir.path()).unwrap();
        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );
        let runner = MockRunner::default();
        let psh = Psh::new(layout.clone(), runner.clone());

        psh.foot_setup().unwrap();

        let commands = runner.commands();
        assert!(commands.iter().any(|cmd| cmd.program == "git"
            && cmd.args.contains(&"clone".to_string())
            && cmd.args.contains(&CREATE_ROBOT_REPO.to_string())));
        assert!(commands
            .iter()
            .any(|cmd| cmd.program == "git" && cmd.args.contains(&LIBCREATE_REPO.to_string())));
        env::remove_var("PSH_ALLOW_MISSING_DEPENDENCIES");
    }

    #[test]
    fn module_action_unknown_module_errors() {
        let repo_dir = tempdir().unwrap();
        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );
        let psh = Psh::new(layout, MockRunner::default());
        let err = psh
            .module_action("unknown", ModuleAction::Setup)
            .unwrap_err();
        assert!(err.to_string().contains("Unknown module"));
    }

    #[test]
    fn install_ros2_invokes_install_script() {
        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");
        let repo_dir = tempdir().unwrap();
        let scripts_dir = repo_dir.path().join("scripts");
        fs::create_dir_all(&scripts_dir).unwrap();
        let script_path = scripts_dir.join("install_ros2.sh");
        fs::write(&script_path, b"#!/bin/bash\nexit 0\n").unwrap();
        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );
        let runner = MockRunner::default();
        let psh = Psh::new(layout, runner.clone());

        let mut args = Ros2Args::default();
        args.script = Some(script_path.clone());
        args.distro = Some("jazzy".to_string());

        let result = psh.install_ros2(&args);
        assert!(result.is_ok(), "install_ros2 should execute successfully");

        let commands = runner.commands();
        assert_eq!(commands.len(), 1);
        let command = &commands[0];
        assert_eq!(command.program, "bash");
        assert!(command
            .args
            .iter()
            .any(|arg| arg == script_path.to_string_lossy().as_ref()));
        env::remove_var("PSH_ALLOW_MISSING_DEPENDENCIES");
    }
}
