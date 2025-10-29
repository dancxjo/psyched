from setuptools import setup

package_name = "psyched_gps"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/psyched_gps.launch.py"],
        ),
    ],
    install_requires=["setuptools", "gps3"],
    zip_safe=True,
    maintainer="Psyched",
    maintainer_email="tdreed@gmail.com",
    description="u-blox GPS node using gpsd, publishing NavSatFix and related topics",
    license="MIT",
    entry_points={
        "console_scripts": [
            "psyched_gps_node = psyched_gps.node:main",
        ],
    },
)
