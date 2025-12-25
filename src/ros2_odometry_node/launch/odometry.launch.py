import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('ros2_odometry_node'),
        'config',
        'odometry_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_odometry_node',
            executable='ros2_odometry_node',
            name='ros2_odometry_node',
            output='screen',
            parameters=[config_file] 
        )
    ])
