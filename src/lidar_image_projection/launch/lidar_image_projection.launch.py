from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 lidar_image_projection 节点"""

    pkg_share = get_package_share_directory('lidar_image_projection')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='lidar_image_projection',
            executable='lidar_image_projection_node',
            name='lidar_image_projection',
            output='screen',
            parameters=[config_file],
        ),
    ])
