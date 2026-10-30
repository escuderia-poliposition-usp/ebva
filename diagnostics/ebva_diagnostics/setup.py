from pathlib import Path
from setuptools import find_packages, setup

name = 'ebva_diagnostics'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('launch', 'config', 'docs'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        data.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=data,
      install_requires=['setuptools', 'rich'], zip_safe=True,
      tests_require=['pytest'],
      maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Componentes ebva_diagnostics extraidos para execucao ROS independente', license='MIT',
      entry_points={'console_scripts': ['odometry_viewer = ebva_diagnostics.odometry_viewer:main', 'map_odon_distance = ebva_diagnostics.map_odom_distance:main']})
