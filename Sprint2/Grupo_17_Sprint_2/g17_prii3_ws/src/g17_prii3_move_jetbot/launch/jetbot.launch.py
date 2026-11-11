"""Lanzamiento físico local al robot: Ethernet se usa para SSH."""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Configura el entorno y lanza el chasis y el controlador fisico elegido.
def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('mode', default_value='draw_number', choices=[
            'draw_number', 'collision_avoidance', 'obstacle_avoidance']),
        DeclareLaunchArgument('start_chassis', default_value='true'),
        DeclareLaunchArgument('domain_id', default_value='17'),
        DeclareLaunchArgument('localhost_only', default_value='1', choices=['0', '1']),
        DeclareLaunchArgument('start_paused', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/wheel/odometry'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
    ]
    numeric = {
        'robot_radius': '0.18', 'safety_margin': '0.03',
        'max_linear': '0.12', 'min_linear': '0.06',
        'max_angular': '0.65', 'min_angular': '0.18',
        'linear_scale': '1.0', 'scan_yaw_offset': '0.0',
        'scan_x': '0.0', 'scan_y': '0.0',
    }
    arguments += [DeclareLaunchArgument(key, default_value=value)
                  for key, value in numeric.items()]
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=float)
                  for key in numeric}
    parameters.update({key: ParameterValue(LaunchConfiguration(key), value_type=bool)
                       for key in ('start_paused',)})
    parameters.update({key: LaunchConfiguration(key)
                       for key in ('odom_topic', 'scan_topic', 'cmd_vel_topic')})
    parameters['use_sim_time'] = False
    chassis = PathJoinSubstitution([
        FindPackageShare('jetbot_pro_ros2'), 'launch', 'jetbot.py'])
    return LaunchDescription(arguments + [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('localhost_only')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(chassis),
                                 condition=IfCondition(LaunchConfiguration('start_chassis'))),
        Node(package='g17_prii3_move_jetbot', executable=LaunchConfiguration('mode'),
             output='screen', parameters=[parameters]),
    ])
