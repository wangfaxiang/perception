from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 lidar_image_projection 节点：前雷达(front) + 后雷达(rear) 各一个实例"""

    pkg_share = get_package_share_directory('lidar_image_projection')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达 (E1R)
        Node(
            package='lidar_image_projection',
            executable='lidar_image_projection_node',
            name='lidar_image_projection_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达 (E1R)
        Node(
            package='lidar_image_projection',
            executable='lidar_image_projection_node',
            name='lidar_image_projection_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
