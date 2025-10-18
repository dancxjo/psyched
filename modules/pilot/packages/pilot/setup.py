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
    ],
    install_requires=['setuptools', 'jsonschema>=4.0'],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='tdreed@gmail.com',
    description='Feeling + will integrator producing FeelingIntent messages and memory writes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pilot_node = pilot.node:main',
        ],
    },
)
