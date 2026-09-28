from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps/office', glob('maps/office/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 team',
    maintainer_email='dev@example.com',
    description='Hardware-free simulation of the G1 web interfaces',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mock_robot = g1_sim.mock_robot:main',
        ],
    },
)
