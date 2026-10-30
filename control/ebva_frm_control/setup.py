from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_frm_control'
resources = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('data', 'config', 'launch', 'docs'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        resources.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))

_extra_resources = []
for _folder in ('config', 'data', 'launch'):
    for _parent in sorted({_p.parent for _p in Path(_folder).rglob('*') if _p.is_file() and '__pycache__' not in _p.parts and _p.suffix != '.pyc'}):
        _extra_resources.append(('share/ebva_frm_control/' + _parent.as_posix(), [str(_p) for _p in sorted(_parent.iterdir()) if _p.is_file() and _p.suffix != '.pyc']))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=(resources) + _extra_resources,
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Controladores FRM e contratos de amostragem.', license='MIT',
      entry_points={'console_scripts': ['point_sample = ebva_frm_control.point_sample:main', 'scan_average = ebva_frm_control.scan_average:main']})
