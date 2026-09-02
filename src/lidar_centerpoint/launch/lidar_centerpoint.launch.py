from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 lidar_centerpoint：单节点同时接收前/后雷达点云并做一次推理"""

    pkg_share = get_package_share_directory('lidar_centerpoint')
    config_file = os.path.join(pkg_share, 'config', 'lidar_centerpoint.param.yaml')

    return LaunchDescription([
        Node(
            package='lidar_centerpoint',
            executable='lidar_centerpoint_node',
            name='lidar_centerpoint',
            output='screen',
            parameters=[config_file],
        ),
    ])
