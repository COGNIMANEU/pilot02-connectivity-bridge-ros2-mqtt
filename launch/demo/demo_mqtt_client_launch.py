from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('mqtt_client')
    config_path = os.path.join(pkg_dir, 'config', 'demo', 'config.yaml')

    return LaunchDescription([
        Node(
            package='mqtt_client',
            executable='mqtt_client',
            name='mqtt_client',
            output='screen',
            parameters=[config_path]
        )
    ])
