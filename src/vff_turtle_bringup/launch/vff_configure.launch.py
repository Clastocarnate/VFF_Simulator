import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # Spawn the goal turtle (top right corner)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 9.0, y: 9.0, theta: 0.0, name: "goal"}'],
            output='screen'
        ),

        # Launch the Python node to handle turtle spawning and movement
        Node(
            package='turtle_controller',  # Ensure this package is created and contains the script
            executable='turtle_vff_controller.py',
            name='turtle_vff_controller',
            output='screen'
        ),
    ])
