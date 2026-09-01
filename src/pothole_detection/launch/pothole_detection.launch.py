from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 pothole_detection 节点：前雷达(front) + 后雷达(rear) 各一个实例"""

    pkg_share = get_package_share_directory('pothole_detection')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达 (E1R)
        Node(
            package='pothole_detection',
            executable='pothole_detection_node',
            name='pothole_detection_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达 (E1R)
        Node(
            package='pothole_detection',
            executable='pothole_detection_node',
            name='pothole_detection_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
