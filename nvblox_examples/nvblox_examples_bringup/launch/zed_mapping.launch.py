from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import SEMSEGNET_INPUT_IMAGE_WIDTH, \
    SEMSEGNET_INPUT_IMAGE_HEIGHT, NVBLOX_CONTAINER_NAME
import os

# Launch 관련 모듈 추가
from launch_ros.actions import LifecycleNode
from launch.actions import ExecuteProcess

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
    
    args.add_arg(
        'nav2_params_file',
        '/home/aimy/navigation_ws/src/aimy_main/2_navigation/aimy_navigation_bringup/config/nav2/nav2_mapping.yaml',
        description='Full path to the ROS2 parameters file to use for Nav2.',
        cli=True)
    
    args.add_arg(
        'map',
        '',
        description='Map file not required, starting with an empty map.',
        cli=True)
    
    
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))

   
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
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
                'camera': NvbloxCamera.zed,
            }))
    
     # map_saver_server 노드 추가
    # actions.append(
    #     LifecycleNode(
    #         package='nav2_map_server',
    #         executable='map_saver_server',
    #         name='map_saver',
    #         namespace='',  # 필요한 경우 변경
    #         output='screen',
    #         parameters=[],
    #     )
    # )

    # # map_saver의 lifecycle 명령 추가
    # actions.append(
    #     ExecuteProcess(
    #         cmd=['ros2', 'lifecycle', 'set', '/map_saver', 'configure'],
    #         output='screen'
    #     )
    # )
    # actions.append(
    #     ExecuteProcess(
    #         cmd=['ros2', 'lifecycle', 'set', '/map_saver', 'activate'],
    #         output='screen'
    #     )
    # )

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
