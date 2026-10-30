from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_cbf_control'
resources = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('data', 'config', 'launch', 'docs'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        resources.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))

_extra_resources = []
for _folder in ('config', 'data', 'launch'):
    for _parent in sorted({_p.parent for _p in Path(_folder).rglob('*') if _p.is_file() and '__pycache__' not in _p.parts and _p.suffix != '.pyc'}):
        _extra_resources.append(('share/ebva_cbf_control/' + _parent.as_posix(), [str(_p) for _p in sorted(_parent.iterdir()) if _p.is_file() and _p.suffix != '.pyc']))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=(resources) + _extra_resources,
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Filtros CBF com modelos e contratos explícitos.', license='MIT',
      entry_points={'console_scripts': ['yaw_rate_margin_stamped = ebva_cbf_control.alternate_model_node:main_margin', 'steering_tangent_stamped = ebva_cbf_control.alternate_model_node:main_tangent', 'yaw_rate_stamped = ebva_cbf_control.yaw_rate_node:main', 'yaw_rate_filter = ebva_cbf_control.yaw_rate.cbf_headway_beams:main', 'yaw_rate_margin_filter = ebva_cbf_control.yaw_rate_margin.cbf_headway_beams:main', 'steering_tangent_filter = ebva_cbf_control.steering_tangent.cbf_headway_beams:main']})
