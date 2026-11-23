"""Reconstrucción 2.5D del PGM: paredes observadas y un ArUco en la entrada.

El mapa no contiene alturas ni objetos de zonas desconocidas. Se usan paredes
verticales de 1 m; no se presenta esta reconstrucción como el mundo original.
"""
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory as share
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

from .bringup import include
from .map_tools import Map

_temporary = []


def box(link, name, xyz, size, color='0.7 0.7 0.7 1', collision=True, yaw=0.):
    for kind in ('collision', 'visual') if collision else ('visual',):
        element = ET.SubElement(link, kind, name=name+'_'+kind)
        ET.SubElement(element, 'pose').text = ' '.join(map(str, (*xyz, 0., 0., yaw)))
        geometry = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
        ET.SubElement(geometry, 'size').text = ' '.join(map(str, size))
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            for field in ('ambient', 'diffuse'):
                ET.SubElement(material, field).text = color
            ET.SubElement(element, 'cast_shadows').text = 'false'


def world_xml(map_path, marker_id, door):
    grid = Map(map_path)
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='f1l3_desde_mapa')
    for model in ('ground_plane', 'sun'):
        ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://'+model
    plugin = ET.SubElement(world, 'plugin', name='estado', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/gazebo'
    ET.SubElement(plugin, 'update_rate').text = '5.0'
    walls = ET.SubElement(world, 'model', name='paredes_f1l3')
    ET.SubElement(walls, 'static').text = 'true'
    link = ET.SubElement(walls, 'link', name='paredes')
    for index, (left, right, top, bottom) in enumerate(grid.rectangles()):
        x, y = grid.world((left+right)/2, (top+bottom)/2)
        box(link, 'pared_'+str(index), (x, y, .5),
            ((right-left)*grid.resolution, (bottom-top)*grid.resolution, 1.), yaw=grid.yaw)
    # Cartel frente a la puerta, con margen para verlo entero desde la cámara.
    # Es visual: representa una marca sostenida y no bloquea el paso.
    panel = ET.SubElement(world, 'model', name='cartel_aruco')
    ET.SubElement(panel, 'static').text = 'true'
    yaw = door[2]
    ET.SubElement(panel, 'pose').text = ' '.join(map(str, (
        door[0]+1.4*math.cos(yaw), door[1]+1.4*math.sin(yaw), 0., 0., 0., yaw)))
    link = ET.SubElement(panel, 'link', name='cartel')
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    pixels = (cv2.aruco.generateImageMarker(dictionary, int(marker_id), 6)
              if hasattr(cv2.aruco, 'generateImageMarker')
              else cv2.aruco.drawMarker(dictionary, int(marker_id), 6))
    box(link, 'fondo', (0., 0., .22), (.008, .40, .40), '1 1 1 1', False)
    for row, col in zip(*np.where(pixels == 0)):
        box(link, f'pixel_{row}_{col}', (-.007, (2.5-col)*.05, .22+(2.5-row)*.05),
            (.004, .0501, .0501), '0 0 0 1', False)
    return ET.tostring(sdf, encoding='unicode')


def simulation_nodes(marker_id, gui, map_path, route, external_world=''):
    temporary = TemporaryDirectory(prefix='g17_gazebo_')
    _temporary.append(temporary)
    folder = Path(temporary.name)
    world = Path(external_world).resolve() if external_world else folder/'f1l3.world'
    if external_world:
        if not world.is_file():
            raise ValueError('No se encuentra el mundo SDF indicado')
    else:
        world.write_text(world_xml(map_path, marker_id, route['door']))
    gazebo = Path(share('turtlebot3_gazebo'))
    model = ET.parse(gazebo/'models'/'turtlebot3_waffle'/'model.sdf')
    for image in model.findall('.//sensor/camera/image'):
        image.find('width').text, image.find('height').text = '640', '480'
    sdf = folder/'robot.sdf'
    model.write(sdf)
    urdf = (gazebo/'urdf'/'turtlebot3_waffle.urdf').read_text()
    model_path = str(gazebo/'models')+':'+os.environ.get('GAZEBO_MODEL_PATH', '')
    initial = route['initial']
    return [SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
            SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
            SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', os.environ.get('LIBGL_ALWAYS_SOFTWARE', '1')),
            SetEnvironmentVariable('LP_NUM_THREADS', os.environ.get('LP_NUM_THREADS', '4')),
            include('gazebo_ros', 'gazebo.launch.py', world=world,
                    gui=str(gui).lower(), verbose='false'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[dict(use_sim_time=True, robot_description=urdf)]),
            Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'waffle', '-file', str(sdf), '-z', '0.01',
                            '-x', str(initial[0]), '-y', str(initial[1]), '-Y', str(initial[2])])]
