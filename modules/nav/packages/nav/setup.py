from setuptools import setup

package_name = 'nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Psyched Dev',
    maintainer_email='devnull@example.com',
    description='Navigation and SLAM bringup package',
    license='Apache-2.0',
    tests_require=['pytest'],
)
