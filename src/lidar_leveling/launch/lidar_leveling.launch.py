from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 lidar_leveling 节点：前雷达(front) + 后雷达(rear) 各一个实例"""

    pkg_share = get_package_share_directory('lidar_leveling')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达调平
        Node(
            package='lidar_leveling',
            executable='lidar_leveling_node',
            name='lidar_leveling_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达调平
        Node(
            package='lidar_leveling',
            executable='lidar_leveling_node',
            name='lidar_leveling_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
