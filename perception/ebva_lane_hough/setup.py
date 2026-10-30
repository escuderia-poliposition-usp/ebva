from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_lane_hough'
resources = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('data', 'config', 'launch'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        resources.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=resources,
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True, maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Métodos alternativos de percepção de faixas.', license='MIT',
      entry_points={'console_scripts': []})
