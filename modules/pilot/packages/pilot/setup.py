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
        ('share/' + package_name + '/static', [
            'pilot/static/index.html',
            'pilot/static/assets.css',
            'pilot/static/app.js',
        ]),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn[standard]', 'pydantic', 'psutil', 'rosidl_runtime_py'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='tdreed@gmail.com',
    description='Pilot web interface with joystick control for robot movement via WebSocket and cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pilot_backend = pilot.main:main',
            'host_health = pilot.host_health_node:main',
            'pilot_ap = pilot.ap_node:main',
        ],
    },
)