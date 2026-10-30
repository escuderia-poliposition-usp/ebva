from pathlib import Path
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ebva_navigation'


_owned_resources=[]
for _folder in ('config','launch','docs'):
    for _parent in sorted({_p.parent for _p in Path(_folder).rglob('*') if _p.is_file() and '__pycache__' not in _p.parts and _p.suffix!='.pyc'}):
        _owned_resources.append(('share/ebva_navigation/'+_parent.as_posix(),[str(_p) for _p in _parent.iterdir() if _p.is_file() and _p.suffix!='.pyc']))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ]) + _owned_resources,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poliposition',
    maintainer_email='ebva.usp@gmail.com',
    description='Nav2 integration glue for the Ackermann car.',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann_node = ebva_navigation.cmd_vel_to_ackermann_node:main',
        ],
    },

)
