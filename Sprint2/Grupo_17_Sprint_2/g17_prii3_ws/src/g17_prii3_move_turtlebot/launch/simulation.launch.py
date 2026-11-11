"""Gazebo, TurtleBot3 Waffle y uno de los tres controladores del sprint."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Lanza Gazebo, el controlador y los elementos opcionales de la simulacion.
def generate_launch_description():
    # Localiza los recursos y prepara el mundo, el robot y el obstaculo opcional.
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    package_share = get_package_share_directory('g17_prii3_move_turtlebot')
    world_launch = os.path.join(
        gazebo_share, 'launch', 'empty_world.launch.py')
    cube = os.path.join(package_share, 'models', 'cubo.sdf')
    spawn_cube = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        name='spawn_cube_g17', output='screen',
        arguments=['-entity', 'cubo_g17', '-file', cube,
                   '-x', '0.0', '-y', '-0.9', '-z', '0.25'],
    )
    delete_cube = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/delete_entity',
             'gazebo_msgs/srv/DeleteEntity', "{name: 'cubo_g17'}"],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='draw_number',
            choices=[
                'draw_number', 'collision_avoidance', 'obstacle_avoidance'],
            description='Controlador que ejecutará el TurtleBot3'),
        DeclareLaunchArgument(
            'spawn_cube', default_value='false',
            description='Inserta un cubo delante del primer trazo'),
        DeclareLaunchArgument(
            'auto_remove_cube', default_value='false',
            description='Retira el cubo 20 segundos después de insertarlo'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Abre RViz para ver /g17/drawing y /scan'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(world_launch)),
        Node(
            # El argumento mode elige cual de los tres ejecutables arrancar.
            package='g17_prii3_move_turtlebot',
            executable=LaunchConfiguration('mode'),
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        TimerAction(
            period=2.0,
            condition=IfCondition(LaunchConfiguration('spawn_cube')),
            actions=[spawn_cube],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_cube,
                on_exit=[TimerAction(period=20.0, actions=[delete_cube])],
            ),
            condition=IfCondition(LaunchConfiguration('auto_remove_cube')),
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': True}],
        ),
    ])
