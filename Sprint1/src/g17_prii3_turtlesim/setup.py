import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'g17_prii3_turtlesim'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='alejandrossalido',

    maintainer_email='alejandro@todo.todo',

    description='Nodo ROS 2 para dibujar el numero 17 con turtlesim',

    license='TODO: License declaration',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'draw_17 = g17_prii3_turtlesim.draw_17:main',
        ],
    },
)
