from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 range_image_segmentation 节点：前雷达(front) + 后雷达(rear) 各一个实例。
    输入接调平点云，输出地面点云 + 非地面障碍物聚类。"""

    pkg_share = get_package_share_directory('range_image_segmentation')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达：输入 /front/rslidar_points_leveled
        Node(
            package='range_image_segmentation',
            executable='range_image_segmentation_node',
            name='range_image_segmentation_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达：输入 /rear/rslidar_points_leveled
        Node(
            package='range_image_segmentation',
            executable='range_image_segmentation_node',
            name='range_image_segmentation_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
