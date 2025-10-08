from setuptools import find_packages, setup

package_name = 'memory'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'neo4j>=5.15,<6', 'qdrant-client>=1.7,<2'],
    zip_safe=True,
    maintainer='Psyched Maintainers',
    maintainer_email='pete@psyched.local',
    description='Memory module connecting Qdrant vector storage with Neo4j.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'memory_node = memory.node:main',
        ],
    },
)
