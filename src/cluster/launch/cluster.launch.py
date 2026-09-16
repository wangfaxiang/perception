from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 cluster 节点：前雷达(front) + 后雷达(rear) 各一个实例，
    输入接 ransac 地面过滤输出的 no_ground 点云。

    每个实例按车辆运动方向定向检测（前进只测前、后退只测后、停车/原地旋转双向均测），
    并发布「单雷达原始障碍物告警」~/obstacle_warning_raw；契约话题
    /perception/obstacle_warning 与 /perception/stop_command 由 perception_common 包的
    共用融合节点发布（与边坡检测共用）。

    本 launch 默认把该融合节点（warning_fusion）也一起拉起，方便单条命令完成
    「聚类 → 原始告警 → 契约话题/急停」整条链路；它需要边坡侧两路原始告警才能完整融合，
    只跑 cluster 时边坡两路会按断流（source_timeout_s）计入安全默认值。

    若已另行 `ros2 launch perception_common warning_fusion.launch.py`，
    或调试态已起融合实例，用 start_warning_fusion:=false 避免两个实例重复发布契约话题：

        ros2 launch cluster cluster.launch.py start_warning_fusion:=false

    参数：跨包共享阈值（obstacle_warn_dist 等）在 perception_common/config/params.yaml
    的 /** 段（唯一真源），两个文件都要加载。
    """

    pkg_share = get_package_share_directory('cluster')
    shared_share = get_package_share_directory('perception_common')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    shared_file = os.path.join(shared_share, 'config', 'params.yaml')

    start_warning_fusion = LaunchConfiguration('start_warning_fusion')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_warning_fusion',
            default_value='true',
            description='是否同时启动 perception_common 的共用报警融合节点 warning_fusion'
                        '（/perception/{edge_warning,obstacle_warning,stop_command} 的唯一发布者）。'
                        '已单独起过融合节点时置 false，避免重复发布契约话题/重复联动急停。',
        ),
        # 前雷达聚类（输入: /front/no_ground，输出: /front/box/cluster/...）
        Node(
            package='cluster',
            executable='cluster',
            name='cluster_front',
            output='screen',
            parameters=[shared_file, config_file],
        ),
        # 后雷达聚类（输入: /rear/no_ground，输出: /rear/box/cluster/...）
        Node(
            package='cluster',
            executable='cluster',
            name='cluster_rear',
            output='screen',
            parameters=[shared_file, config_file],
        ),
        # 共用报警融合节点（订阅本节点的两路 ~/obstacle_warning_raw + 边坡两路原始告警，
        # 发布契约三话题）；节点名 warning_fusion 需与 perception_common 参数文件的段名一致
        # Node(
        #     package='perception_common',
        #     executable='warning_fusion_node',
        #     name='warning_fusion',
        #     output='screen',
        #     parameters=[shared_file],
        #     condition=IfCondition(start_warning_fusion),
        # ),
    ])
