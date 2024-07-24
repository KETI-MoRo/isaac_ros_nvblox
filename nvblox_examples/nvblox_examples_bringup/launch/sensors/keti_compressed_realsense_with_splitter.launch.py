# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', NVBLOX_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')

    # Config file
    config_file = lu.get_path('nvblox_examples_bringup', 'config/sensors/keti_realsense.yaml')

    # Splitter node
    realsense_splitter_node = ComposableNode(
        namespace='camera',
        name='realsense_splitter_node',
        package='realsense_splitter',
        plugin='nvblox::KetiRealsenseSplitterNode',
        parameters=[{
            'input_qos': 'SENSOR_DATA',
            'output_qos': 'SENSOR_DATA'
            # 'input_qos': 'RELIABLE',
            # 'output_qos': 'RELIABLE'
        }],
        remappings=[
            ('input/infra_1', '/camera/infra1/image_rect_raw'), # keti  여기서  ros2 topic echo /camera/realsense_splitter_node/output/depth  문제문제문제
            ('input/infra_1_metadata', '/camera/infra1/metadata'),
            ('input/infra_2', '/camera/infra2/image_rect_raw'), # keti
            ('input/infra_2_metadata', '/camera/infra2/metadata'),
            ('input/depth', '/camera/depth/image_rect_raw'),    # keti
            ('input/depth_metadata', '/camera/depth/metadata'),
            ('input/pointcloud', '/camera/depth/color/points'),
            ('input/pointcloud_metadata', '/camera/depth/metadata'),
        ])

    # Driver node
    realsense_node = ComposableNode(
        # namespace='camera', # ASUS
        namespace='', # Desktop
        remappings=[
            # ('/camera/infra1/image_rect_raw', '/camera/infra1/image_rect_raw_dy'),
            # ('/camera/infra2/image_rect_raw', '/camera/infra2/image_rect_raw_dy'),
            # ('/camera/depth/image_rect_raw','/camera/depth/image_rect_raw_dy'),
            ('/camera/color/image_raw','/camera/color/image_raw_dy'),
            
            
            ],
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[config_file])

    actions = args.get_launch_actions()
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))))
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            # [realsense_splitter_node, realsense_node], # keti
            [realsense_splitter_node, realsense_node], # keti
            # [realsense_node], # keti

        ))

    return LaunchDescription(actions)
