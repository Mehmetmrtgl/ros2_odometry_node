import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_odometry_node')
    common_config_path = os.path.join(pkg_share, 'config', 'common.yaml')

    drive_type = "MECANUM"  
    
    try:
        with open(common_config_path, 'r') as f:
            content = yaml.safe_load(f)
            drive_type = content['ros2_odometry_node']['ros__parameters']['drive_type']
    except Exception as e:
        print(f"--- Warning: Could not read common.yaml, using default: {e} ---")

    specific_config_name = f"{drive_type.lower()}_params.yaml"
    specific_config_path = os.path.join(pkg_share, 'config', specific_config_name)

    print(f"--- Selected Drive Mode: {drive_type} ---")
    print(f"--- Loading Specific Config: {specific_config_path} ---")

    return LaunchDescription([
        Node(
            package='ros2_odometry_node',
            executable='ros2_odometry_node',
            name='ros2_odometry_node',
            output='screen',
            parameters=[common_config_path, specific_config_path]
        )
    ])