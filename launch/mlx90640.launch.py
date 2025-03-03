from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mlx90640',
            executable='mlx90640_node',
            name='mlx90640_node',
            output='screen',
            parameters=['config/mlx90640_config.yaml'] 
        )
    ])