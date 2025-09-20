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
    install_requires=['setuptools', 'pyaudio', 'webrtcvad'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='you@example.com',
    description='PyAudio-based microphone capture with silence detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ear_node = ear.pyaudio_ear_node:main',
            'vad_node = ear.vad_node:main',
        ],
    },
)
