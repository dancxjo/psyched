from setuptools import setup

package_name = 'wifi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'zeroconf'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='tdreed@gmail.com',
    description='Wi-Fi AP user-space setup and ROS2 status publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_ap_manager = wifi.ap_manager:main',
        ],
    },
)
