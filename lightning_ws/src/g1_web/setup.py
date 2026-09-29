import os
from glob import glob

from setuptools import setup

package_name = 'g1_web'


def www_files():
    """把 www/ 下的前端文件按目录结构装到 share/g1_web/www。"""
    out = {}
    for path in glob('www/**/*', recursive=True):
        if os.path.isfile(path):
            out.setdefault(os.path.join('share', package_name, os.path.dirname(path)), []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'serve.py']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + www_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 team',
    maintainer_email='dev@example.com',
    description='G1 web console (static files)',
    license='Apache-2.0',
)
