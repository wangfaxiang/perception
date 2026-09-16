"""感知链路一键启动（bringup）——前后雷达各一路，整条链路一条命令拉齐

环节（对应 README 的启动步骤）：
  0) 数据源（二选一）
       use_bag:=true   回放 rosbag：rosbag2_stone_front / rosbag2_stone_rear
       use_bag:=false  启动 rslidar_sdk 真实雷达驱动
       （两个 bag 里的话题名已是 /front/rslidar_points、/rear/rslidar_points，无需 remap）
  1) lidar_leveling       点云调平      /<side>/rslidar_points -> /<side>/rslidar_points_leveled
  2) pothole_detection    边坡/沟壑检测  -> /<side>/ditch_cloud、/<side>/ditch_line
  3) ground_segmentation  地面点去除     -> /<side>/no_ground
  4) cluster              欧式聚类障碍物 -> /<side>/box/cluster/euclidean/marker
  5) warning_fusion       报警融合（/perception/{edge_warning,obstacle_warning,stop_command}
                          契约三话题的唯一发布者；因此 cluster.launch 里那份融合节点必须关掉）
  6) rviz2 ×2             可视化        front.rviz / rear.rviz

用法：
  cd <workspace>
  source /opt/ros/galactic/setup.bash && source install/setup.bash

  # 默认：回放 bag（前/后各一个，--loop）+ 全链路 + 两个 rviz
  ros2 launch perception_common perception_bringup.launch.py

  # 真实雷达驱动（不下发 bag）
  ros2 launch perception_common perception_bringup.launch.py use_bag:=false

  # 换 bag / 不循环 / 调延迟
  ros2 launch perception_common perception_bringup.launch.py \\
      front_bag:=rosbag2_obstacle_front rear_bag:=rosbag2_obstacle_rear \\
      bag_loop:=false bag_delay_s:=5.0

  # 只跑链路不看图（例如已有独立 rviz / 无显示环境）
  ros2 launch perception_common perception_bringup.launch.py start_rviz:=false

  # 工作目录不在 <workspace> 根时，显式指定（bag 与 .rviz 都按它解析）
  ros2 launch perception_common perception_bringup.launch.py \\
      workspace_dir:=/home/promote/wfx/work/lin_gong/perception

注意：
  * bag 与 rviz 配置默认按「运行 ros2 launch 时的当前目录」解析；本文件默认取 cwd，
    所以请在工作区根目录执行（或用 workspace_dir:= 覆盖）。
  * 延迟 bag 播放（bag_delay_s，默认 3s）是为了让下游节点先就位；bag 播放是循环的，
    即使丢开头几帧也不影响后续观察。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(pkg, *parts):
    """参数包 share 目录下的路径。"""
    return os.path.join(get_package_share_directory(pkg), *parts)


def _include(pkg, launch_file, launch_arguments=None):
    """包含子包的 launch 文件。"""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share(pkg, 'launch', launch_file)),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _warn(msg):
    """Galactic 的 launch 没有 LogWarn，用 LogInfo + 前缀代替。"""
    return LogInfo(msg=f'[bringup][警告] {msg}')


def _launch_setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def is_true(name):
        return value(name).strip().lower() in ('1', 'true', 'yes', 'on')

    def abs_path(path):
        return path if os.path.isabs(path) else os.path.join(workspace, path)

    workspace = value('workspace_dir') or os.getcwd()

    actions = [
        LogInfo(msg=f'[bringup] 工作目录: {workspace}'),
    ]

    # ---------- 0) 数据源：rosbag 回放 or 雷达驱动（二选一） ----------
    if is_true('use_bag'):
        bags = []
        for label, arg_name in (('前雷达', 'front_bag'), ('后雷达', 'rear_bag')):
            bag_path = abs_path(value(arg_name))
            if not os.path.isdir(bag_path):
                actions.append(_warn(f'{label} rosbag 不存在，已跳过: {bag_path}'))
                continue
            cmd = ['ros2', 'bag', 'play', bag_path]
            if is_true('bag_loop'):
                cmd.append('--loop')
            bags.append(ExecuteProcess(cmd=cmd, cwd=workspace, output='screen'))

        if bags:
            delay = float(value('bag_delay_s'))
            detail = '（--loop 循环）' if is_true('bag_loop') else ''
            actions.append(
                LogInfo(msg=f'[bringup] 数据源: rosbag 回放{detail}，延迟 {delay:.1f}s 开始播放')
            )
            actions.append(TimerAction(period=delay, actions=bags))
        else:
            actions.append(_warn('没有可播放的 rosbag，链路将收不到点云'))
    else:
        actions.append(LogInfo(msg='[bringup] 数据源: rslidar_sdk 雷达驱动'))
        actions.append(
            Node(
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                name='rslidar_sdk_node',
                output='screen',
                # 留空 = 用包内 config/config.yaml（与 `ros2 launch rslidar_sdk start.py` 一致）
                parameters=[{'config_path': value('rslidar_config_path')}],
            )
        )

    # ---------- 1~5) 感知链路 ----------
    actions.append(LogInfo(msg='[bringup] 1/5 点云调平: lidar_leveling'))
    actions.append(_include('lidar_leveling', 'lidar_leveling.launch.py'))

    actions.append(LogInfo(msg='[bringup] 2/5 边坡/沟壑检测: pothole_detection'))
    actions.append(_include('pothole_detection', 'pothole_detection.launch.py'))

    actions.append(LogInfo(msg='[bringup] 3/5 地面点去除: ground_segmentation'))
    actions.append(_include('ground_segmentation', 'ransac_ground_filter.launch.py'))

    # 融合节点由本文件统一启动，cluster 自带的那份必须关掉（契约话题只能有一个发布者）
    actions.append(LogInfo(msg='[bringup] 4/5 聚类障碍物检测: cluster（自带融合已关闭）'))
    actions.append(
        _include('cluster', 'cluster.launch.py', {'start_warning_fusion': 'false'})
    )

    actions.append(LogInfo(msg='[bringup] 5/5 报警融合: warning_fusion'))
    actions.append(_include('perception_common', 'warning_fusion.launch.py'))

    # ---------- 6) 可视化 ----------
    if is_true('start_rviz'):
        rviz_nodes = []
        for side, arg_name in (('front', 'front_rviz_config'), ('rear', 'rear_rviz_config')):
            rviz_path = abs_path(value(arg_name))
            if not os.path.isfile(rviz_path):
                actions.append(_warn(f'rviz 配置不存在，已跳过 {side}: {rviz_path}'))
                continue
            rviz_nodes.append(
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name=f'rviz2_{side}',
                    output='log',
                    arguments=['-d', rviz_path],
                )
            )
        if rviz_nodes:
            actions.append(LogInfo(msg='[bringup] 可视化: rviz2 front + rear'))
            actions.append(TimerAction(period=float(value('rviz_delay_s')), actions=rviz_nodes))
    else:
        actions.append(LogInfo(msg='[bringup] 可视化: 已关闭 (start_rviz:=false)'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # ---- 数据源 ----
        DeclareLaunchArgument(
            'use_bag', default_value='true',
            description='true=回放 rosbag（离线）；false=启动 rslidar_sdk 真实雷达驱动',
        ),
        DeclareLaunchArgument(
            'front_bag', default_value='rosbag2_stone_front',
            description='前雷达 rosbag 目录（相对路径按 workspace_dir 解析）',
        ),
        DeclareLaunchArgument(
            'rear_bag', default_value='rosbag2_stone_rear',
            description='后雷达 rosbag 目录（相对路径按 workspace_dir 解析）',
        ),
        DeclareLaunchArgument(
            'bag_loop', default_value='true',
            description='rosbag 是否 --loop 循环播放',
        ),
        DeclareLaunchArgument(
            'bag_delay_s', default_value='3.0',
            description='延迟多少秒后开始回放 bag（等下游节点就位）',
        ),
        DeclareLaunchArgument(
            'rslidar_config_path', default_value='',
            description='rslidar_sdk 配置文件路径；留空 = 包内 config/config.yaml',
        ),
        # ---- 路径 ----
        DeclareLaunchArgument(
            'workspace_dir', default_value='',
            description='工作区根目录（bag 与 .rviz 的相对路径基准）；留空 = 当前目录',
        ),
        # ---- 可视化 ----
        DeclareLaunchArgument(
            'start_rviz', default_value='true', description='是否启动 front/rear 两个 rviz2',
        ),
        DeclareLaunchArgument(
            'front_rviz_config', default_value='front.rviz', description='前雷达 rviz 配置',
        ),
        DeclareLaunchArgument(
            'rear_rviz_config', default_value='rear.rviz', description='后雷达 rviz 配置',
        ),
        DeclareLaunchArgument(
            'rviz_delay_s', default_value='3.0', description='延迟多少秒后启动 rviz2',
        ),
        # ---- 组装 ----
        OpaqueFunction(function=_launch_setup),
    ])
