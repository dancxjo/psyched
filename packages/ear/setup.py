from setuptools import setup

package_name = 'ear'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ear.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='you@example.com',
    description='Microphone PCM publisher using arecord, publishing audio_common_msgs/AudioData',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ear_node = ear.node:main',
        ],
    },
)
