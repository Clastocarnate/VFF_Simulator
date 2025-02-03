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

        # Spawn the bot (2.0, 2.0)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 2.0, y: 2.0, theta: 0.0, name: "bot"}'],
            output='screen'
        ),

        # Spawn the goal (8.0, 8.0)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 8.0, y: 8.0, theta: 0.0, name: "goal"}'],
            output='screen'
        ),

        # Spawn the obstacle (should be between bot and goal, e.g., 5.0, 5.0)
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 5.0, y: 5.0, theta: 0.0, name: "obstacle"}'],
            output='screen'
        ),
    ])