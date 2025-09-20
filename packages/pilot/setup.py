from setuptools import setup

package_name = 'pilot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pilot.launch.py']),
        ('share/' + package_name + '/static', ['pilot/static/index.html', 'pilot/static/joystick.js', 'pilot/static/style.css']),
    ],
    install_requires=['setuptools', 'websockets', 'asyncio'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='you@example.com',
    description='Pilot web interface with joystick control for robot movement via WebSocket and cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pilot_node = pilot.node:main',
        ],
    },
)