from setuptools import find_packages, setup

package_name = "hypothalamus"

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
    description="Temperature and humidity telemetry bridge for Pete's hypothalamus module.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hypothalamus_node = hypothalamus.node:main",
        ]
    },
)
