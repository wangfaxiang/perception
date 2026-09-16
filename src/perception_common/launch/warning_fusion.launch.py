from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动感知侧共用「报警融合节点」（契约三话题的唯一发布者）

    输入（两类检测实例的单雷达原始告警）:
      /pothole_detection_{front,rear}/edge_warning_raw   （pothole_detection 包，边坡）
      /cluster_{front,rear}/obstacle_warning_raw         （cluster 包，障碍物）
    输出（契约 §4.1 冻结话题）:
      /perception/edge_warning / /perception/obstacle_warning / /perception/stop_command
    """

    pkg_share = get_package_share_directory('perception_common')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='perception_common',
            executable='warning_fusion_node',
            name='warning_fusion',
            output='screen',
            parameters=[config_file],
        ),
    ])
