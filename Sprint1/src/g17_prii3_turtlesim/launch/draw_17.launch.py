from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Inicia turtlesim y el nodo que dibuja el 17."""

    # Arranca el simulador junto al controlador del grupo.
    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        Node(
            package='g17_prii3_turtlesim',
            executable='draw_17',
            name='draw_17',
            output='screen'
        )

    ])
