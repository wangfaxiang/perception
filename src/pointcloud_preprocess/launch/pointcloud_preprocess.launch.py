from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 pointcloud_preprocess 节点：前雷达(front) + 后雷达(rear) 各一个实例。

    输入调平点云（/front|rear/rslidar_points_leveled），全量扬尘过滤 → 降采样 → 高度平滑，
    输出 /front|rear/leveled_filtered_smoothed 给 elevation_obstacle_detection。
    """

    pkg_share = get_package_share_directory('pointcloud_preprocess')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达预处理
        Node(
            package='pointcloud_preprocess',
            executable='pointcloud_preprocess_node',
            name='pointcloud_preprocess_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达预处理
        Node(
            package='pointcloud_preprocess',
            executable='pointcloud_preprocess_node',
            name='pointcloud_preprocess_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
