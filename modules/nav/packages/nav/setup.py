from setuptools import setup

package_name = 'psyched_nav'

setup(
    name=package_name,
    version='0.1.0',
    # The Python package directory is still `nav` (module imports use `nav.*`),
    # so explicitly install that module while the ROS package/distribution name
    # is `psyched_nav`.
    packages=['nav'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Psyched Dev',
    maintainer_email='devnull@example.com',
    description='Navigation and SLAM bringup package',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Install launch files and params into share/<package> so `ros2 launch` can find them
    data_files=[
        # Put launch files under share/<package>/launch so `ros2 launch` finds them
        (f'share/{package_name}/launch', [
            'nav_bringup.launch.py',
        ]),
        # Install params files
        (f'share/{package_name}/params', [
            'params/nav2_params.yaml',
            'params/rtabmap_params.yaml',
        ]),
    ],
)
