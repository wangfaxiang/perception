from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 pothole_detection：前雷达(front) + 后雷达(rear) 两个检测实例 + 1 个告警融合节点

    检测实例按车辆运动方向定向检测（前进只测前、后退只测后、停车/原地旋转双向均测），
    各自发布原始告警 ~/edge_warning_raw；
    融合节点取等级更高的一路，发布契约话题 /perception/edge_warning。
    """

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
        # 边坡告警融合（/perception/edge_warning 唯一发布者）
        Node(
            package='pothole_detection',
            executable='edge_warning_fusion_node',
            name='edge_warning_fusion',
            output='screen',
            parameters=[config_file],
        ),
    ])
