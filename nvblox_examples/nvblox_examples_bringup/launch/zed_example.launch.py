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

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import SEMSEGNET_INPUT_IMAGE_WIDTH, \
    SEMSEGNET_INPUT_IMAGE_HEIGHT, NVBLOX_CONTAINER_NAME
import os

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '', description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.static,
        # default=NvbloxMode.dynamic,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'people_segmentation',
        default=NvbloxPeopleSegmentation.peoplesemsegnet_vanilla,
        choices=[
            str(NvbloxPeopleSegmentation.peoplesemsegnet_vanilla),
            str(NvbloxPeopleSegmentation.peoplesemsegnet_shuffleseg)
        ],
        description='The  model type of PeopleSemSegNet (only used when mode:=people).',
        cli=True)
    
    # args.add_arg(
    #     'nav2_params_file',
    #     '/home/hyeongu/isaac_ws/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nav2/nav2.yaml',
    #     description='Full path to the ROS2 parameters file to use for Nav2.',
    #     cli=True)
    
    args.add_arg(
        'nav2_params_file',
        '/home/aimy/roboworld_ws/src/aimy_main/2_navigation/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nav2/nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for Nav2.',
        cli=True)
    
    args.add_arg(
        'map',
        '/home/aimy/roboworld_ws/src/aimy_main/2_navigation/isaac_ros_nvblox/map/real_robo_map_v2.yaml',
        description='Map file not required, starting with an empty map.',
        cli=True)
    
    # args.add_arg(
    #     'filtered_map',
    #     '/home/aimy/roboworld_ws/src/aimy_main/2_navigation/isaac_ros_nvblox/map/robo_map_filtered.yaml',
    #     description='Map file not required, starting with an empty map.',
    #     cli=True)

    
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))


    # ZED
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/sensors/zed.launch.py',
            launch_arguments={'container_name': NVBLOX_CONTAINER_NAME},
            condition=UnlessCondition(lu.is_valid(args.rosbag))))

    
    # Nav2 bringup
    if args.map:
        actions.append(
            lu.include(
                'nav2_bringup',
                'launch/bringup_launch.py',
                launch_arguments={
                    'params_file': args.nav2_params_file,
                    'map': args.map,
                    # 'filtered_map': args.filtered_map,
                    'use_sim_time': 'false'
                }))
    else:
        actions.append(
            lu.include(
                'nav2_bringup',
                'launch/bringup_launch.py',
                launch_arguments={
                    'params_file': args.nav2_params_file,
                    'use_sim_time': 'false'
                }))
    
    # Nvblox
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/perception/nvblox.launch.py',
    #         launch_arguments={
    #             'container_name': NVBLOX_CONTAINER_NAME,
    #             'mode': args.mode,
    #             'camera': NvbloxCamera.zed,
    #         }))

    # Visualization
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/visualization/visualization.launch.py',
    #         launch_arguments={
    #             'mode': args.mode,
    #             'camera': NvbloxCamera.zed
    #         }))

    # Container
    actions.append(lu.component_container(NVBLOX_CONTAINER_NAME, log_level=args.log_level))

    return LaunchDescription(actions)
