import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    front_driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_ls_front.yaml')
    rear_driver_config = os.path.join(get_package_share_directory('lslidar_driver'),'config','lslidar_ls_rear.yaml')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    front_driver_node = ""
    rear_driver_node = ""
    ros_version = p.communicate()[0]

    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        front_driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='front', # 与对应yaml文件中命名空间一致
                                    output='screen',
                                    parameters=[front_driver_config],
                                    )

        rear_driver_node = LifecycleNode(package='lslidar_driver',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    node_namespace='rear', # 与对应yaml文件中命名空间一致
                                    output='screen',
                                    parameters=[rear_driver_config],
                                    )

    else:
        front_driver_node = Node(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='front', # 与对应yaml文件中命名空间一致
                                    parameters=[front_driver_config],
                                    output='screen'
                                    )

        rear_driver_node = Node(package='lslidar_driver',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    namespace='rear', # 与对应yaml文件中命名空间一致
                                    parameters=[rear_driver_config],
                                    output='screen'
                                    )

    # 雷达数据流监测节点（与驱动一起启动）：跟踪前后雷达是否有数据/是否断流，
    # 发布 /front/lidar_status、/rear/lidar_status（lidar_status_monitor/msg/LidarStatus）
    front_monitor_node = Node(package='lidar_status_monitor',
                              executable='lidar_status_monitor_node',
                              name='lidar_status_monitor_node',
                              namespace='front',
                              parameters=[{
                                  'lidar_name': 'front',
                                  'input_topic': 'rslidar_points',
                                  'status_topic': 'lidar_status',
                              }],
                              output='screen')

    rear_monitor_node = Node(package='lidar_status_monitor',
                             executable='lidar_status_monitor_node',
                             name='lidar_status_monitor_node',
                             namespace='rear',
                             parameters=[{
                                 'lidar_name': 'rear',
                                 'input_topic': 'rslidar_points',
                                 'status_topic': 'lidar_status',
                             }],
                             output='screen')

    return LaunchDescription([
        front_driver_node,
        rear_driver_node,   # 后雷达驱动（启用后雷达时，同时取消下面 rear_monitor_node 注释）
        front_monitor_node,
        rear_monitor_node,
    ])
