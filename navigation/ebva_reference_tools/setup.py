from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_reference_tools'
resources = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('data', 'config', 'launch'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        resources.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=resources,
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Coleta, geração e publicação de referências.', license='MIT',
      entry_points={'console_scripts': ['centerline_logger = ebva_reference_tools.centerline_logger:main', 'clearance_logger = ebva_reference_tools.clearance_logger:main', 'reference_points_logger = ebva_reference_tools.reference_points_logger:main', 'trajectory_generator = ebva_reference_tools.trajectory_generator:main', 'waypoint_collector = ebva_reference_tools.waypoint_collector:main', 'extract_path = ebva_reference_tools.extract_path:main', 'publish_path = ebva_reference_tools.publish_path:main']})
