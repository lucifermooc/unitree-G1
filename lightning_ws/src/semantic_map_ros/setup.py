from glob import glob

from setuptools import find_packages, setup

package_name = 'semantic_map_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docker', glob('docker/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 team',
    maintainer_email='dev@example.com',
    description='Semantic map: sentence -> waypoint_node point -> /nav_to_pose',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'semantic_map_server = semantic_map_ros.semantic_map_server:main',
        ],
    },
)
