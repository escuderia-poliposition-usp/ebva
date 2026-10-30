# Copyright 2026 EBVA (Escuderia Brasileira de Veículos Autônomos)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
from setuptools import find_packages, setup

package_name = 'ebva_sign_detection'

_extra_resources = []
for _folder in ("config", "launch"):
    for _parent in sorted({
        _p.parent for _p in Path(_folder).rglob("*")
        if _p.is_file() and _p.suffix != ".pyc"
    }):
        _extra_resources.append((
            "share/ebva_sign_detection/" + _parent.as_posix(),
            [str(_p) for _p in _parent.iterdir() if _p.is_file()],
        ))

_owned_resources = []
for _folder in ('config', 'launch', 'docs'):
    for _parent in sorted({
        _p.parent for _p in Path(_folder).rglob('*')
        if _p.is_file() and '__pycache__' not in _p.parts and _p.suffix != '.pyc'
    }):
        _owned_resources.append((
            'share/ebva_sign_detection/' + _parent.as_posix(),
            [str(_p) for _p in _parent.iterdir() if _p.is_file() and _p.suffix != '.pyc'],
        ))
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=([
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + _extra_resources) + _owned_resources,
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='poliposition',
    maintainer_email='tumang@usp.br',
    description='Traffic sign detection for F1TENTH team Escuderia Poliposition',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = ebva_sign_detection.yolo_node:main',
        ],
    },
)
