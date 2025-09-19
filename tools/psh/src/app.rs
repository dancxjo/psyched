use anyhow::{bail, Context, Result};
use std::env;
use std::ffi::OsString;
use std::fs;
use std::io;
use std::path::PathBuf;

use crate::cli::{Commands, HostCommands, ModuleArgs, ModuleCommands, Ros2Args};
use crate::command_runner::{CommandRunner, CommandSpec};
use crate::host::HostConfig;
use crate::layout::{copy_release_binary, user_install_dir, Layout};
use crate::util::{ensure_tool, psh_binary, set_executable_permissions, sh_quote};

const REPO_URL: &str = "https://github.com/dancxjo/psyched.git";

pub struct App<R: CommandRunner> {
    layout: Layout,
    runner: R,
}

impl App<crate::command_runner::RealCommandRunner> {
    pub fn default() -> Result<Self> {
        let layout = Layout::detect()?;
        Ok(Self::new(layout, crate::command_runner::RealCommandRunner))
    }
}

impl<R: CommandRunner> App<R> {
    pub fn new(layout: Layout, runner: R) -> Self {
        Self { layout, runner }
    }

    pub fn execute(&self, command: Commands) -> Result<()> {
        match command {
            Commands::Install => self.install(),
            Commands::Clone => self.clone_repo(),
            Commands::Build => self.build(),
            Commands::Update => self.update(),
            Commands::Env => self.env(),
            Commands::Ros2(args) => self.install_ros2(&args),
            Commands::Remove => self.remove(),
            Commands::Host { command } => match command {
                HostCommands::Apply(args) => self.host_apply(&args.hostname),
            },
            Commands::Module { command } => self.handle_module_command(command),
        }
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
                    match fs::create_dir_all(parent) {
                        Ok(()) => {}
                        Err(err) => {
                            if err.kind() == io::ErrorKind::PermissionDenied {
                                self.runner.run(
                                    &CommandSpec::new("sudo")
                                        .arg("mkdir")
                                        .arg("-p")
                                        .arg(parent.to_string_lossy().to_string()),
                                )?;
                            } else {
                                return Err(err).with_context(|| {
                                    format!("unable to create {}", parent.display())
                                });
                            }
                        }
                    }
                }
            }

            if !self.layout.repo_path.exists() {
                match fs::create_dir_all(&self.layout.repo_path) {
                    Ok(()) => {}
                    Err(err) => {
                        if err.kind() == io::ErrorKind::PermissionDenied {
                            let path_str = self.layout.repo_path.to_string_lossy().to_string();
                            self.runner.run(
                                &CommandSpec::new("sudo")
                                    .arg("mkdir")
                                    .arg("-p")
                                    .arg(path_str.clone()),
                            )?;
                            let user = env::var("USER").unwrap_or_else(|_| "root".to_string());
                            let owner = format!("{user}:{user}");
                            self.runner.run(
                                &CommandSpec::new("sudo")
                                    .arg("chown")
                                    .arg("-R")
                                    .arg(owner)
                                    .arg(path_str),
                            )?;
                        } else {
                            return Err(err).with_context(|| {
                                format!("unable to create {}", self.layout.repo_path.display())
                            });
                        }
                    }
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

    fn env(&self) -> Result<()> {
        let script = self.layout.setup_env_script();
        if !script.exists() {
            bail!(
                "{} does not exist. Run `psh clone` first to fetch the repository or provide PSH_REPO_DIR.",
                script.display()
            );
        }

        let workspace = self.layout.repo_path.to_string_lossy();
        let distro = crate::util::ros_distro();
        let command = format!(
            "WORKSPACE_PATH={} ROS_DISTRO={} bash {}",
            sh_quote(&workspace),
            sh_quote(&distro),
            sh_quote(&script.to_string_lossy()),
        );

        self.runner.run(
            &CommandSpec::new("bash")
                .arg("-lc")
                .arg(command)
                .cwd(&self.layout.repo_path),
        )?;
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

        let distro = args.distro.clone().unwrap_or_else(crate::util::ros_distro);

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

    fn host_apply(&self, hostname: &str) -> Result<()> {
        let config = HostConfig::load(self.layout.hosts_dir(), hostname)?;
        for module in &config.modules {
            if module.setup {
                self.module_action(&module.name, ModuleAction::Setup)?;
            }
            if module.launch {
                self.module_action(&module.name, ModuleAction::Launch)?;
            }
            if module.teardown {
                self.module_action(&module.name, ModuleAction::Teardown)?;
            }
        }
        Ok(())
    }

    fn module_action(&self, name: &str, action: ModuleAction) -> Result<()> {
        ensure_tool("bash", "Install bash so module scripts can run.")?;
        match action {
            ModuleAction::Launch => self.launch_module(name),
            ModuleAction::Setup | ModuleAction::Teardown => {
                self.run_module_script(name, action)?;
                Ok(())
            }
        }
    }

    fn handle_module_command(&self, command: ModuleCommands) -> Result<()> {
        match command {
            ModuleCommands::Setup(args) => self.run_for_modules(&args, ModuleAction::Setup),
            ModuleCommands::Remove(args) => self.run_for_modules(&args, ModuleAction::Teardown),
            ModuleCommands::Launch(args) => self.run_for_modules(&args, ModuleAction::Launch),
        }
    }

    fn run_for_modules(&self, args: &ModuleArgs, action: ModuleAction) -> Result<()> {
        for module in &args.modules {
            self.module_action(module, action)?;
        }
        Ok(())
    }

    fn run_module_script(&self, name: &str, action: ModuleAction) -> Result<()> {
        let script = self.module_script_path(name, action);
        if !script.exists() {
            bail!("Unknown module action: no script at {}", script.display());
        }

        self.runner.run(
            &CommandSpec::new("bash")
                .arg(script.to_string_lossy())
                .cwd(&self.layout.repo_path),
        )?;
        Ok(())
    }

    fn launch_module(&self, name: &str) -> Result<()> {
        let script = self.module_script_path(name, ModuleAction::Launch);
        if !script.exists() {
            bail!("Unknown module action: no script at {}", script.display());
        }

        let distro = crate::util::ros_distro();
        let psh = psh_binary().with_context(|| "resolving current psh executable")?;
        let env_eval = format!(
            "source <(WORKSPACE_PATH={workspace} ROS_DISTRO={distro} PSH_ENV_MODE=print {psh} env)",
            workspace = sh_quote(&self.layout.repo_path.to_string_lossy()),
            distro = sh_quote(&distro),
            psh = sh_quote(&psh.to_string_lossy()),
        );

        let launch_script = format!("./modules/{name}/launch.sh", name = name);
        let launch_cmd = format!(
            "{env_eval} && {launch}",
            env_eval = env_eval,
            launch = sh_quote(&launch_script),
        );

        self.runner.run(
            &CommandSpec::new("bash")
                .arg("-lc")
                .arg(launch_cmd)
                .cwd(&self.layout.repo_path),
        )?;
        Ok(())
    }

    fn module_script_path(&self, name: &str, action: ModuleAction) -> PathBuf {
        let script_name = match action {
            ModuleAction::Setup => "setup.sh",
            ModuleAction::Teardown => "teardown.sh",
            ModuleAction::Launch => "launch.sh",
        };
        self.layout.modules_dir().join(name).join(script_name)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ModuleAction {
    Setup,
    Teardown,
    Launch,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command_runner::{CommandOutput, CommandSpec};
    use crate::layout::Layout;
    use std::cell::RefCell;
    use std::env;
    use std::rc::Rc;
    use std::sync::{Mutex, OnceLock};
    use tempfile::tempdir;

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

    fn env_lock() -> &'static Mutex<()> {
        static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
        LOCK.get_or_init(|| Mutex::new(()))
    }

    fn layout() -> Layout {
        let repo_dir = tempdir().unwrap();
        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        )
    }

    #[test]
    fn install_copies_release_binary() {
        let layout = layout();
        let runner = MockRunner::default();
        let app = App::new(layout.clone(), runner);

        let release = layout.release_binary();
        fs::create_dir_all(release.parent().unwrap()).unwrap();
        fs::write(&release, b"fake binary").unwrap();

        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");
        app.install().expect("install succeeds");

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
        let app = App::new(layout.clone(), runner);

        let release = layout.release_binary();
        fs::create_dir_all(release.parent().unwrap()).unwrap();
        fs::write(&release, b"fallback binary").unwrap();

        let home_dir = tempdir().unwrap();
        let original_home = env::var("HOME").ok();
        env::set_var("HOME", home_dir.path());

        let result = app.install();
        assert!(result.is_ok(), "install should succeed with fallback");

        let fallback_path = home_dir
            .path()
            .join(Layout::default_install_subdir_for_tests())
            .join("psh");
        assert!(
            fallback_path.exists(),
            "fallback binary should be installed"
        );
        assert!(
            !primary_dir.join("psh").exists(),
            "primary path should remain untouched"
        );
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
    fn module_action_unknown_module_errors() {
        let layout = layout();
        let app = App::new(layout, MockRunner::default());
        let err = app
            .module_action("unknown", ModuleAction::Setup)
            .unwrap_err();
        assert!(err.to_string().contains("Unknown module action"));
    }

    #[test]
    fn install_ros2_invokes_install_script() {
        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_ALLOW_MISSING_DEPENDENCIES", "1");
        let repo_dir = tempdir().unwrap();
        let scripts_dir = repo_dir.path().join("tools");
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
        let app = App::new(layout, runner.clone());

        let mut args = Ros2Args::default();
        args.script = Some(script_path.clone());
        args.distro = Some("jazzy".to_string());

        let result = app.install_ros2(&args);
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

    #[test]
    fn module_command_invokes_script() {
        let repo_dir = tempdir().unwrap();
        let modules_dir = repo_dir.path().join("modules");
        let script_dir = modules_dir.join("dummy");
        fs::create_dir_all(&script_dir).unwrap();
        let script_path = script_dir.join("setup.sh");
        fs::write(&script_path, b"#!/bin/bash\nexit 0\n").unwrap();

        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );

        let runner = MockRunner::default();
        let app = App::new(layout, runner.clone());

        let args = ModuleArgs {
            modules: vec!["dummy".to_string()],
        };

        assert!(app.run_for_modules(&args, ModuleAction::Setup).is_ok());

        let commands = runner.commands();
        assert_eq!(commands.len(), 1);
        assert_eq!(commands[0].program, "bash");
        assert!(commands[0]
            .args
            .iter()
            .any(|arg| arg.ends_with("modules/dummy/setup.sh")));
    }

    #[test]
    fn module_launch_sources_via_psh_env_before_invoking_script() {
        let _guard = env_lock().lock().unwrap();
        env::set_var("PSH_SELF", "/usr/bin/psh-test");

        let repo_dir = tempdir().unwrap();
        let modules_dir = repo_dir.path().join("modules");
        let script_dir = modules_dir.join("dummy");
        fs::create_dir_all(&script_dir).unwrap();
        let launch_path = script_dir.join("launch.sh");
        fs::write(&launch_path, b"#!/bin/bash\nros2 launch foo bar\n").unwrap();

        let tools_dir = repo_dir.path().join("tools");
        fs::create_dir_all(&tools_dir).unwrap();
        let setup_env = tools_dir.join("setup_env.sh");
        fs::write(&setup_env, b"#!/bin/bash\nexit 0\n").unwrap();

        let install_dir = tempdir().unwrap();
        let systemd_dir = tempdir().unwrap();
        let layout = Layout::new(
            repo_dir.path().into(),
            install_dir.path().into(),
            systemd_dir.path().into(),
        );

        let runner = MockRunner::default();
        let app = App::new(layout, runner.clone());

        let args = ModuleArgs {
            modules: vec!["dummy".to_string()],
        };

        assert!(app.run_for_modules(&args, ModuleAction::Launch).is_ok());

        let commands = runner.commands();
        assert_eq!(commands.len(), 1);
        let command = &commands[0];
        assert_eq!(command.program, "bash");
        assert_eq!(command.args.first().map(String::as_str), Some("-lc"));
        let body = command.args.get(1).expect("launch command body");
        assert!(body.contains("PSH_ENV_MODE=print '/usr/bin/psh-test' env"));
        assert!(body.contains("modules/dummy/launch.sh"));

        env::remove_var("PSH_SELF");
    }

    #[test]
    fn env_invokes_setup_env_script() {
        let layout = layout();
        let tools_dir = layout.repo_path.join("tools");
        fs::create_dir_all(&tools_dir).unwrap();
        let script = tools_dir.join("setup_env.sh");
        fs::write(&script, b"#!/bin/bash\nexit 0\n").unwrap();

        let runner = MockRunner::default();
        let app = App::new(layout, runner.clone());
        assert!(app.env().is_ok());

        let commands = runner.commands();
        assert_eq!(commands.len(), 1);
        let command = &commands[0];
        assert_eq!(command.program, "bash");
        assert_eq!(command.args.first().map(String::as_str), Some("-lc"));
        let body = command.args.get(1).expect("env command body");
        assert!(body.contains("setup_env.sh"));
    }
}
