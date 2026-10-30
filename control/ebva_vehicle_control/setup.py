from pathlib import Path
from setuptools import find_packages, setup
name = "ebva_vehicle_control"
resources = [
    ("share/ament_index/resource_index/packages", ["resource/" + name]),
    ("share/" + name, ["package.xml", "README.md"]),
]
for folder in ("launch", "docs"):
    for parent in sorted({p.parent for p in Path(folder).rglob("*") if p.is_file()}):
        resources.append(("share/" + name + "/" + parent.as_posix(),
                          [str(p) for p in parent.iterdir() if p.is_file()]))

_extra_resources = []
for _folder in ('config', 'data', 'launch'):
    for _parent in sorted({_p.parent for _p in Path(_folder).rglob('*') if _p.is_file() and '__pycache__' not in _p.parts and _p.suffix != '.pyc'}):
        _extra_resources.append(('share/ebva_vehicle_control/' + _parent.as_posix(), [str(_p) for _p in sorted(_parent.iterdir()) if _p.is_file() and _p.suffix != '.pyc']))
setup(name=name, version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=(resources) + _extra_resources, install_requires=["setuptools", "pyserial>=3.5,<4", "PyYAML"],
      tests_require=["pytest"], zip_safe=True, license="MIT",
      maintainer="Poliposition", maintainer_email="ebva.usp@gmail.com",
      description="Separated RC acquisition, manual authority and bounded VESC backend",
      entry_points={'console_scripts': ['rc_acquisition = ebva_vehicle_control.nodes:rc_main', 'keyboard_adapter = ebva_vehicle_control.keyboard_adapter:main', 'key_teleop_terminal = ebva_vehicle_control.key_teleop_terminal:main', 'command_authority = ebva_vehicle_control.nodes:authority_main', 'vesc_backend = ebva_vehicle_control.nodes:backend_main', 'ftg_pd_adapter = ebva_vehicle_control.ftg_pd_adapter:main', 'cbf_yaw_rate_margin_adapter = ebva_vehicle_control.cbf_alternate_adapter:main_margin', 'cbf_steering_tangent_adapter = ebva_vehicle_control.cbf_alternate_adapter:main_tangent', 'cbf_yaw_rate_adapter = ebva_vehicle_control.cbf_yaw_rate_adapter:main', 'effort_steering_adapter = ebva_vehicle_control.effort_steering_node:main', 'supervisor_effort_adapter = ebva_vehicle_control.supervisor_effort_node:main', 'frm_effort_adapter = ebva_vehicle_control.frm_effort_node:main', 'ackermann_adapter = ebva_vehicle_control.ackermann_node:main', 'navigation_adapter = ebva_vehicle_control.navigation_adapter:main']})
