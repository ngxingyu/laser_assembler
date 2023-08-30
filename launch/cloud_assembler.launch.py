#!/usr/bin/python3.6
#
# Copyright 2018 Open Source Robotics Foundation, Inc.
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

    config = os.path.join(
        get_package_share_directory('laser_assembler'),
        'config',
        'cloud_assembler.yaml'
    )

    print(config)
    laser_assembler = Node(
        package='laser_assembler',
        executable='cloud2_scan_assembler',
        name='cloud2_scan_assembler',
        # prefix='gdb -ex run --args',
        parameters=[config,
        #             {
        #     "fixed_frame": "wamv/wamv/base_link",
        # }
        ],
        output='screen',
        remappings=[('cloud', '/wamv/merged_cloud')]
    )

    return LaunchDescription([
        laser_assembler
    ])
