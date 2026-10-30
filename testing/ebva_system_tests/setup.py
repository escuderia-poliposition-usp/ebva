from pathlib import Path
from setuptools import find_packages, setup
name = 'ebva_system_tests'
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
      tests_require=['pytest'], entry_points={'console_scripts': ['supervisor_replay = ebva_system_tests.supervisor_replay:main', 'manual_core_replay = ebva_system_tests.manual_core_replay:main', 'stanley_frenet_replay = ebva_system_tests.stanley_frenet_replay:main', 'alternate_cbf_replay = ebva_system_tests.alternate_cbf_replay:main', 'effort_steering_replay = ebva_system_tests.effort_steering_replay:main', 'frm_replay = ebva_system_tests.frm_replay:main', 'ftg_pd_replay = ebva_system_tests.ftg_pd_replay:main', 'csv_frenet_replay = ebva_system_tests.csv_frenet_replay:main', 'waypoint_pp_replay = ebva_system_tests.waypoint_pp_replay:main', 'cbf_yaw_rate_replay = ebva_system_tests.cbf_yaw_rate_replay:main', 'imu_prior_replay = ebva_system_tests.imu_prior_replay:main', 'lane_perception_replay = ebva_system_tests.lane_perception_replay:main', 'cbf_pipeline_replay = ebva_system_tests.cbf_pipeline_replay:main', 'bench_probe = ebva_system_tests.bench_probe:main', 'hough_demo = ebva_system_tests.hough_demo:main', 'lane_control_replay = ebva_system_tests.lane_replay:main', 'run_all = ebva_system_tests.campaigns:main']})
