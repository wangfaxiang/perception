from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # front / rear 两个实例，分别跟踪前后雷达点云数据流。
    # 输入话题按 namespace 解析：namespace=front 且 input_topic=rslidar_points
    #   -> 订阅 /front/rslidar_points，发布 /front/lidar_status
    front_monitor = Node(
        package='lidar_status_monitor',
        executable='lidar_status_monitor_node',
        name='lidar_status_monitor_node',
        namespace='front',
        parameters=[{
            'lidar_name': 'front',
            'input_topic': 'rslidar_points',
            'status_topic': 'lidar_status',
            'frame_id': 'base_link',
            'timeout_s': 1.0,
            'publish_hz': 10.0,
        }],
        output='screen',
    )

    rear_monitor = Node(
        package='lidar_status_monitor',
        executable='lidar_status_monitor_node',
        name='lidar_status_monitor_node',
        namespace='rear',
        parameters=[{
            'lidar_name': 'rear',
            'input_topic': 'rslidar_points',
            'status_topic': 'lidar_status',
            'frame_id': 'base_link',
            'timeout_s': 1.0,
            'publish_hz': 10.0,
        }],
        output='screen',
    )

    return LaunchDescription([
        front_monitor,
        rear_monitor,
    ])
