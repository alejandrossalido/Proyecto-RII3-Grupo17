import os
from glob import glob

from setuptools import setup


package_name = 'g17_prii3_move_jetbot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='grupo17',
    maintainer_email='grupo17@example.invalid',
    description='Sprint 2: dibujo del 17 y obstáculos en JetBot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'diagnose_robot = g17_prii3_move_jetbot.diagnostics:main',
            'draw_number = g17_prii3_move_jetbot.controller:main_draw',
            'collision_avoidance = '
            'g17_prii3_move_jetbot.controller:main_collision',
            'obstacle_avoidance = '
            'g17_prii3_move_jetbot.controller:main_obstacle',
        ],
    },
)
