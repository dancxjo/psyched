"""Tests describing the nav module packaging expectations."""

from importlib import import_module
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parents[3]
MODULE_TOML = MODULE_ROOT / "module.toml"


def test_python_package_imports() -> None:
    """The python distribution should be importable via `psyched_nav`."""

    package = import_module("psyched_nav")
    assert package.__name__ == "psyched_nav"


def test_python_package_directory_matches_name() -> None:
    """The source directory should mirror the distribution name."""

    assert PACKAGE_ROOT.name == "psyched_nav"


def test_module_installs_nav2_amcl_without_rtabmap() -> None:
    """Ensure the module actions install Nav2 with AMCL but avoid RTAB-Map."""

    config = MODULE_TOML.read_text()
    assert "ros-${ROS_DISTRO}-nav2-amcl" in config
    assert "ros-${ROS_DISTRO}-nav2-bringup" in config
    assert "rtabmap" not in config.lower()


def test_module_builds_psyched_nav_package() -> None:
    """The optional build step should target the `psyched_nav` package."""

    config = MODULE_TOML.read_text()
    assert "--packages-select psyched_nav" in config
