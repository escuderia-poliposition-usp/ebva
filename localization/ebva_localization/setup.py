from pathlib import Path
from setuptools import find_packages, setup

name = 'ebva_localization'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml', 'README.md'])]
for folder in ('launch', 'config', 'docs'):
    for parent in sorted({p.parent for p in Path(folder).rglob('*') if p.is_file()}):
        data.append(('share/' + name + '/' + parent.as_posix(), [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))
setup(name=name, version='0.1.0', packages=find_packages(exclude=['test']), data_files=data,
      install_requires=['setuptools'], zip_safe=True,
      tests_require=['pytest'],
      maintainer='Poliposition', maintainer_email='ebva.usp@gmail.com',
      description='Componentes ebva_localization extraidos para execucao ROS independente', license='MIT',
      entry_points={'console_scripts': ['state_tf_audit = ebva_localization.state_tf_audit:main', 'odometry_pose = ebva_localization.odometry_pose:main', 'pose_republisher = ebva_localization.pose_republisher_from_slam_node:main', 'base_link_pose = ebva_localization.baselink_pose_publisher:main', 'odometry_pose_tf = ebva_localization.robot_pose_tf_publisher:main', 'imu_odometry = ebva_localization.imu_odometry:main', 'initial_pose = ebva_localization.initial_pose:main', 'legacy_imu_prior = ebva_localization.legacy_imu_prior:main', 'slam_map_saver = ebva_localization.slam_map_saver:main', 'pose_recorder = ebva_localization.pose_recorder_node:main', 'save_slam_map_once = ebva_localization.save_slam_map_once:main']})
