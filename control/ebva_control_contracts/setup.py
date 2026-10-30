from setuptools import find_packages, setup
name='ebva_control_contracts'
setup(name=name,version='0.1.0',packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),('share/'+name,['package.xml','README.md'])],
      install_requires=['setuptools'],tests_require=['pytest'],zip_safe=True,
      maintainer='Poliposition',maintainer_email='ebva.usp@gmail.com',
      description='Pure measurement validity contracts for EBVA control consumers',license='MIT')
