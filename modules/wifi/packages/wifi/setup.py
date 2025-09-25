from setuptools import setup

package_name = 'wifi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}_setup'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Wi-Fi AP user-space setup and ROS2 status publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wifi_setup = wifi_setup.main:main',
        ],
    },
)
