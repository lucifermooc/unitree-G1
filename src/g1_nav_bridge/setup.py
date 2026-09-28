from glob import glob
from setuptools import find_packages, setup


package_name = 'g1_nav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'ODOMETRY.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 team',
    maintainer_email='robot@example.com',
    description='G1 navigation data bridges with a package-local Unitree API client.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'sport_to_odom = g1_nav_bridge.sport_to_odom:main',
        'tf_to_current_pose = g1_nav_bridge.tf_to_current_pose:main',
    ]},
)
