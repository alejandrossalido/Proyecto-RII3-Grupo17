"""Un launch con modos de la práctica; parámetros Humble más ajustes del grupo."""
import shutil
from pathlib import Path
from tempfile import TemporaryDirectory

import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from .route import load_route

# Conserva los YAML temporales mientras vive el proceso launch.
_temporary = []


def include(package, launch, **arguments):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(Path(share(package)) / 'launch' / launch)),
        launch_arguments={key: str(value) for key, value in arguments.items()}.items())


def merge(target, updates):
    for key, value in updates.items():
        if isinstance(value, dict):
            merge(target.setdefault(key, {}), value)
        else:
            target[key] = value


def prepare(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    mode = value('mode')
    common = Path(share('g17_prii3_nav_common'))
    package = 'g17_prii3_nav_turtlebot'
    folder = Path(share(package))
    route_path = Path(value('route') or folder / 'config' / 'poses.yaml').resolve()
    route = load_route(route_path)
    nodes = []
    mapping = mode == 'mapping'
    map_path = value('map') or str(common/'maps'/'f1l3_nav.yaml')
    if not mapping and not Path(map_path).is_file():
        raise RuntimeError('Indicar map:=/ruta/absoluta/f1l3.yaml; primero crear el mapa')
    temporary = TemporaryDirectory(prefix='g17_sprint3_')
    _temporary.append(temporary)
    temp = Path(temporary.name)
    if not mapping:
        # El parser de parámetros ROS/C++ de Humble altera escapes Unicode de
        # RewrittenYaml. El proyecto contiene «3º»: usar copias temporales ASCII.
        original = Path(map_path).resolve()
        with original.open() as stream:
            map_data = yaml.safe_load(stream)
        image = Path(map_data['image'])
        if not image.is_absolute():
            image = original.parent / image
        copied = temp / ('map_image' + image.suffix)
        shutil.copyfile(image, copied)
        map_data['image'] = str(copied)
        map_path = temp / 'map.yaml'
        map_path.write_text(yaml.safe_dump(map_data))
    if value('start_simulator') == 'true':
        from .simulation import simulation_nodes
        nodes += simulation_nodes(value('marker_id'), value('gui') == 'true',
                                  map_path, route, value('world'))
    with open(Path(share('nav2_bringup')) / 'params' / 'nav2_params.yaml') as stream:
        nav = yaml.safe_load(stream)
    with open(common / 'config' / 'navigation.yaml') as stream:
        merge(nav, yaml.safe_load(stream))
    base, odom, scan = 'base_footprint', '/odom', '/scan'
    # Sustituye las mismas claves en todos los nodos/costmaps, sin duplicar YAML.
    def replace(data):
        for key, item in data.items():
            if isinstance(item, dict):
                replace(item)
            elif key == 'use_sim_time':
                data[key] = True
            elif key in ('base_frame_id', 'robot_base_frame'):
                data[key] = base
            elif key == 'odom_topic':
                data[key] = odom
            elif key == 'scan_topic' or (key == 'topic' and item == '/scan'):
                data[key] = scan
    replace(nav)
    # Solo la simulación conoce automáticamente su posición inicial exacta.
    nav['amcl']['ros__parameters'].update(
        set_initial_pose=value('start_simulator') == 'true',
        initial_pose=dict(zip(('x', 'y', 'yaw'), map(float, route['initial']))))
    params = temp / 'nav2.yaml'
    if mapping:
        with open(Path(share('slam_toolbox')) / 'config' / 'mapper_params_online_async.yaml') as stream:
            slam = yaml.safe_load(stream)
        slam['slam_toolbox']['ros__parameters'].update(
            use_sim_time=True, base_frame=base, scan_topic=scan,
            minimum_travel_distance=0.1, minimum_travel_heading=0.1,
            map_update_interval=1.0)
        nav.update(slam)
    params.write_text(yaml.safe_dump(nav))
    nodes.append(include('nav2_bringup', 'bringup_launch.py',
                         map=Path(map_path).resolve() if not mapping else '',
                         slam=str(mapping), params_file=params,
                         use_sim_time='true', use_composition='True'))
    if mode in ('predefined', 'autonomous'):
        nodes.append(Node(package=package, executable=mode + '_navigation', output='screen',
                          parameters=[dict(use_sim_time=True, route_file=str(route_path))]))
    if mode == 'autonomous':
        camera = Node(package='g17_prii3_nav_common', executable='aruco', output='screen',
                          parameters=[dict(use_sim_time=True,
                              image_topic=value('image_topic'),
                              output_topic='/g17/aruco_robot', show=value('show_camera') == 'true')])
        # Gazebo debe terminar de crear el sensor antes de conectar lectores.
        nodes.append(TimerAction(period=8.0, actions=[camera]))
    if value('rviz') == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='screen',
                          arguments=['-d', str(common / 'config' / 'sprint3.rviz')],
                          parameters=[dict(use_sim_time=True)]))
    return nodes


def make_launch():
    defaults = dict(mode='guided', map='', route='', world='', rviz='true', gui='true',
                    start_simulator='true',
                    image_topic='/camera/image_raw', show_camera='false', marker_id='10')
    arguments = []
    for key, default in defaults.items():
        choices = dict(mode=['mapping', 'guided', 'predefined', 'autonomous'],
                       marker_id=['10', '20', '30'])
        arguments.append(DeclareLaunchArgument(key, default_value=default,
                                               **({'choices': choices[key]} if key in choices else {})))
    return LaunchDescription(arguments + [OpaqueFunction(function=prepare)])
