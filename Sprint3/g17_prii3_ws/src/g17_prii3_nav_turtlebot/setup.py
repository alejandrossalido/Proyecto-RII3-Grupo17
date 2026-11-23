from glob import glob
from setuptools import setup

name = 'g17_prii3_nav_turtlebot'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/launch', glob('launch/*.py')),
                  ('share/' + name + '/config', glob('config/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Grupo 17', maintainer_email='grupo17@example.invalid',
      description='Vision y navegacion del Sprint 3', license='Apache-2.0',
      entry_points={'console_scripts': ['predefined_navigation = g17_prii3_nav_common.mission:predefined_main', 'autonomous_navigation = g17_prii3_nav_common.mission:autonomous_main']})
