from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 pothole_detection 节点"""

    pkg_share = get_package_share_directory('pothole_detection')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='pothole_detection',
            executable='pothole_detection_node',
            name='pothole_detection',
            output='screen',
            parameters=[config_file],
        ),
    ])
