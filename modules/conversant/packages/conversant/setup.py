from setuptools import find_packages, setup

package_name = "conversant"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/conversant.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Psyched Maintainers",
    maintainer_email="pete@psyched.local",
    description="Conversational pacing and concern handling node for Pete.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "conversant_agent = conversant.node:main",
        ],
    },
)
