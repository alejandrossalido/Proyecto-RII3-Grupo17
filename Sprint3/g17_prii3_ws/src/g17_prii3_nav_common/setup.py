from glob import glob
from setuptools import setup

name = 'g17_prii3_nav_common'
setup(name=name, version='0.1.0', packages=[name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + name]),
                  ('share/' + name, ['package.xml']),
                  ('share/' + name + '/config', glob('config/*')),
                  ('share/' + name + '/maps', glob('maps/*'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Grupo 17', maintainer_email='grupo17@example.invalid',
      description='Vision y navegacion del Sprint 3', license='Apache-2.0',
      entry_points={'console_scripts': ['aruco = g17_prii3_nav_common.aruco:main']})
