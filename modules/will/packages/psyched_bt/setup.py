from setuptools import setup

package_name = 'psyched_bt'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.trees'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/brain.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'py_trees',
        'py_trees_ros',
        'psyched_msgs',
    ],
    zip_safe=True,
    maintainer='Psyched',
    maintainer_email='tdreed@gmail.com',
    description='Behaviour tree regimes for Pete\'s cognitive layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'regime_manager = psyched_bt.regime_manager_node:main',
        ],
    },
)
