from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 elevation_obstacle_detection 节点：前雷达(front) + 后雷达(rear) 各一个实例。

    输入接 pointcloud_preprocess 输出的 leveled_filtered_smoothed 点云（全量扬尘过滤 + 降采样 + 高度平滑），
    用局部栅格高程差法重新判定地面障碍物，输出障碍物点云 + 包围盒 MarkerArray 到 rviz。
    """

    pkg_share = get_package_share_directory('elevation_obstacle_detection')
    shared_share = get_package_share_directory('perception_common')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    shared_file = os.path.join(shared_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达：输入 /front/leveled_filtered_smoothed
        Node(
            package='elevation_obstacle_detection',
            executable='elevation_obstacle_detection_node',
            name='elevation_obstacle_detection_front',
            output='screen',
            parameters=[shared_file, config_file],
        ),
        # 后雷达：输入 /rear/leveled_filtered_smoothed
        Node(
            package='elevation_obstacle_detection',
            executable='elevation_obstacle_detection_node',
            name='elevation_obstacle_detection_rear',
            output='screen',
            parameters=[shared_file, config_file],
        ),
    ])
