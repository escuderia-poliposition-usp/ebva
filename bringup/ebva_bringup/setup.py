from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_bringup'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('launch', 'profiles', 'config', 'data', 'docs'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*')
                          if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc'}):
        data.append(('share/' + name + '/' + parent.as_posix(),
                     [str(p) for p in sorted(parent.iterdir()) if p.is_file() and p.suffix != '.pyc']))
setup(name=name, version='0.2.0', packages=find_packages(exclude=['test']),
      data_files=data, install_requires=['setuptools', 'PyYAML'], zip_safe=False,
      maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='EBVA package resources and public ROS interfaces', license='MIT',
      tests_require=['pytest'], entry_points={'console_scripts': []})
