from setuptools import find_packages, setup

package_name = 'ear'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ear.launch.py']),
    ],
    install_requires=['setuptools', 'numpy>=1.24,<3', 'websockets>=12,<13', 'webrtcvad>=2.0.10'],
    extras_require={'faster-whisper': ['faster-whisper>=0.10,<1.0']},
    zip_safe=True,
    maintainer='Psyched Maintainers',
    maintainer_email='pete@psyched.local',
    description='Ear module that collects text from multiple transcription backends.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ear_audio_capture = ear.audio_capture:main',
            'ear_vad = ear.vad_node:main',
            'ear_silence = ear.silence_node:main',
            'ear_transcriber = ear.transcriber_node:main',
        ],
    },
)
