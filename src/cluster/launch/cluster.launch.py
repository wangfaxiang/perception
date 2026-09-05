from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 cluster 节点：前雷达(front) + 后雷达(rear) 各一个实例，
    输入接 ransac 地面过滤输出的 no_ground 点云。"""

    return LaunchDescription([
        # 前雷达聚类（输入: /front/no_ground，输出: /front/box/cluster/...）
        Node(
            package='cluster',
            executable='cluster',
            name='cluster_front',
            output='screen',
            parameters=[{'lidar_name': 'front', 'cluster_tolerance': 0.2,
                         'min_cluster_size': 15, 'max_cluster_size': 100000,
                         'region_rowing': False}],
        ),
        # 后雷达聚类（输入: /rear/no_ground，输出: /rear/box/cluster/...）
        Node(
            package='cluster',
            executable='cluster',
            name='cluster_rear',
            output='screen',
            parameters=[{'lidar_name': 'rear', 'cluster_tolerance': 0.2,
                         'min_cluster_size': 15, 'max_cluster_size': 100000,
                         'region_rowing': False}],
        ),
    ])
