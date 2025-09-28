from setuptools import setup

package_name = 'voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice.launch.py']),
    ],
    # espeak-ng is used for TTS; no Piper support.
    install_requires=['setuptools', 'websockets>=11,<12'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='tdreed@gmail.com',
    description='Voice node subscribing to text and playing via espeak-ng TTS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'voice_node = voice.node:main',
                'voice_volume = voice.volume_node:main',
        ],
    },
)
