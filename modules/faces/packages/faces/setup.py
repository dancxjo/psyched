from setuptools import find_packages, setup

package_name = "faces"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    install_requires=["setuptools", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="Psyched Dev",
    maintainer_email="devnull@example.com",
    description="Face detection and embedding pipeline for the Psyched robots",
    license="Apache-2.0",
    tests_require=["pytest"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/faces"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/face_detector.launch.py"]),
    ],
    entry_points={
        "console_scripts": [
            "face_detector = faces.face_detector_node:main",
        ]
    },
)
