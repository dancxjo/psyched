from setuptools import find_packages, setup

package_name = "psyched_eye"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Psyched Dev",
    maintainer_email="devnull@example.com",
    description="Launch assets for the Psyched eye module Kinect pipeline.",
    license="Apache-2.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/psyched_eye"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/eye.launch.py"]),
    ],
)
