# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 ransac_ground_filter 节点：前雷达(front) + 后雷达(rear) 各一个实例"""

    pkg_share = get_package_share_directory('ground_segmentation')
    config_file = os.path.join(pkg_share, 'config', 'ground_segmentation.param.yaml')

    return LaunchDescription([
        # 前雷达地面滤除（输入: /front/rslidar_points_leveled）
        Node(
            package='ground_segmentation',
            executable='ransac_ground_filter_node',
            name='ransac_ground_filter_front',
            output='screen',
            parameters=[config_file],
        ),
        # 后雷达地面滤除（输入: /rear/rslidar_points_leveled）
        Node(
            package='ground_segmentation',
            executable='ransac_ground_filter_node',
            name='ransac_ground_filter_rear',
            output='screen',
            parameters=[config_file],
        ),
    ])
