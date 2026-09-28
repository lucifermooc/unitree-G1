from setuptools import setup
import os
from glob import glob
package_name = 'aid_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('aid_robot_py/*.py')),
        ('db', ['db/db.sql']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='box',
    maintainer_email='chendongfang@turingvideo.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_manager_node = aid_robot_py.launch_manager:main',
            'map_manager_node = aid_robot_py.map_manager_server:main',
            'map_transform_node = aid_robot_py.map_transform:main',
            'waypoint_manage_node = aid_robot_py.waypoint_manage:main'
        ],
    },
)
