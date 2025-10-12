from setuptools import find_packages, setup

package_name = "viscera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Psyched Maintainers",
    maintainer_email="pete@psyched.local",
    description="Biologically inspired narratives for Pete's internal state.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "viscera_monitor = viscera.cli:main",
            "viscera_host_health = viscera.publisher:main",
        ]
    },
)
