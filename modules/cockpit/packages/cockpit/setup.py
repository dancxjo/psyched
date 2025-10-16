from setuptools import find_packages, setup

package_name = 'cockpit'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    package_data={
        'cockpit': ['frontend/**/*'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'aiohttp>=3.9,<4',
        'PyYAML>=6.0',
        'tomli-w>=1.0',
    ],
    zip_safe=False,
    maintainer='Psyched Maintainers',
    maintainer_email='pete@psyched.local',
    description='Cockpit cockpit server for Psyched robots.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cockpit = cockpit.cli:main',
        ],
    },
)
