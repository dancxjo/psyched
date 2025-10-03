from setuptools import find_packages, setup

package_name = 'voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'websockets>=12,<13'],
    zip_safe=True,
    maintainer='Psyched Maintainers',
    maintainer_email='pete@psyched.local',
    description='Voice module that serializes and plays back spoken text.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_service = voice.cli:main',
        ],
    },
)
