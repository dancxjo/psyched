use crate::layout::Layout;
use crate::util::ros_distro;

pub fn print_shell_setup(layout: &Layout) {
    let distro = ros_distro();
    let repo = layout.repo_path.display();

    println!("To configure a shell for the Psyched workspace, run:");
    println!("  source /etc/profile.d/ros2-defaults.sh");
    println!("  source /opt/ros/{distro}/setup.bash");
    println!("  source {repo}/install/setup.bash");
    println!();
    println!("To persist this configuration:");
    println!("  echo 'source /etc/profile.d/ros2-defaults.sh' >> ~/.bashrc");
    println!("  echo 'source /opt/ros/{distro}/setup.bash' >> ~/.bashrc");
    println!("  echo 'source {repo}/install/setup.bash' >> ~/.bashrc");
}
