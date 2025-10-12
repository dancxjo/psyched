from setuptools import find_packages, setup

package_name = "psyched_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Psyched Dev",
    maintainer_email="devnull@example.com",
    description="Navigation bringup helpers for the Psyched robots",
    license="Apache-2.0",
    data_files=[
        (f"share/{package_name}/launch", [
            "nav_bringup.launch.py",
        ]),
        (f"share/{package_name}/params", [
            "params/nav2_params.yaml",
        ]),
    ],
)
