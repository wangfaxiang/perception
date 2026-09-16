from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 pothole_detection：前雷达(front) + 后雷达(rear) 两个检测实例

    检测实例按车辆运动方向定向检测（前进只测前、后退只测后、停车/原地旋转双向均测），
    各自发布原始告警 ~/edge_warning_raw；契约话题 /perception/edge_warning 与
    /perception/stop_command 由 perception_common 包的共用融合节点发布：

        ros2 launch perception_common warning_fusion.launch.py

    参数：跨包共享阈值（edge_danger_dist / edge_caution_dist 等）在
    perception_common/config/params.yaml 的 /** 段（唯一真源），两个文件都要加载。
    """

    pkg_share = get_package_share_directory('pothole_detection')
    shared_share = get_package_share_directory('perception_common')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    shared_file = os.path.join(shared_share, 'config', 'params.yaml')

    return LaunchDescription([
        # 前雷达 (E1R)
        Node(
            package='pothole_detection',
            executable='pothole_detection_node',
            name='pothole_detection_front',
            output='screen',
            parameters=[shared_file, config_file],
        ),
        # 后雷达 (E1R)
        Node(
            package='pothole_detection',
            executable='pothole_detection_node',
            name='pothole_detection_rear',
            output='screen',
            parameters=[shared_file, config_file],
        ),
    ])
