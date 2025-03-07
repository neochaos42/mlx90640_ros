import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mlx90640'),
        'config',
        'mlx90640_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='mlx90640',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=[config],
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])