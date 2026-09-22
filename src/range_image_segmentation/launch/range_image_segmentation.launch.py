from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 range_image_segmentation 节点：前雷达(front) + 后雷达(rear) 各一个实例。
    输入接调平点云，输出沟壑/悬崖边缘点云 + 折线。

    与 pothole_detection 一致：除本包参数外，同时加载 perception_common/config/params.yaml
    的共享段（所有感知节点统一加载，未声明的键对节点无影响）。"""

    pkg_share = get_package_share_directory('range_image_segmentation')
    shared_share = get_package_share_directory('perception_common')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    shared_file = os.path.join(shared_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达：输入 /front/rslidar_points_leveled
        Node(
            package='range_image_segmentation',
            executable='range_image_segmentation_node',
            name='range_image_segmentation_front',
            output='screen',
            parameters=[shared_file, config_file],
        ),
        # 后雷达：输入 /rear/rslidar_points_leveled
        Node(
            package='range_image_segmentation',
            executable='range_image_segmentation_node',
            name='range_image_segmentation_rear',
            output='screen',
            parameters=[shared_file, config_file],
        ),
    ])
