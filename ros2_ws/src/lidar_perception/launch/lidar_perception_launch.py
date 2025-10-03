from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'lidar_perception',
            executable = 'lidar_perception',
            name = 'lidar_perception',
            output = 'screen'

        )
    ])