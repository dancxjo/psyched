from setuptools import find_packages, setup

package_name = "psyched_eye"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=["setuptools", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="Psyched Dev",
    maintainer_email="devnull@example.com",
    description="Utility nodes for Psyched eye module camera management.",
    license="Apache-2.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/psyched_eye"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/eye.launch.py"]),
    ],
    entry_points={
        "console_scripts": [
            "usb_camera_node = psyched_eye.usb_camera_node:main",
            "faces_router_node = psyched_eye.faces_router_node:main",
        ],
    },
)
