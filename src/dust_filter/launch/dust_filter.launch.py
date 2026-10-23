from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 dust_filter 扬尘过滤节点：前雷达(front) + 后雷达(rear) 各一个实例"""

    pkg_share = get_package_share_directory('dust_filter')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达去扬尘
        Node(
            package='dust_filter',
            executable='dust_filter_node',
            name='dust_filter_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达去扬尘
        Node(
            package='dust_filter',
            executable='dust_filter_node',
            name='dust_filter_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
