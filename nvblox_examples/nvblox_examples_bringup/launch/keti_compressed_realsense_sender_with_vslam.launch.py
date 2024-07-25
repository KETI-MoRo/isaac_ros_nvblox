from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera, NvbloxPeopleSegmentation
from nvblox_ros_python_utils.nvblox_constants import SEMSEGNET_INPUT_IMAGE_WIDTH, \
    SEMSEGNET_INPUT_IMAGE_HEIGHT, NVBLOX_CONTAINER_NAME
    
    
# from launch import LaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import SetParameter
# import os
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg(
        'rosbag', 'None', description='Path to rosbag (running on sensor if not set).', cli=True)
    args.add_arg('rosbag_args', '', description='Additional args for ros2 bag play.', cli=True)
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg(
        'mode',
        default=NvbloxMode.static,
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
        'navigation',
        True,
        description='Whether to enable nav2 for navigation in Isaac Sim.',
        cli=True)
    actions = args.get_launch_actions()

    # Globally set use_sim_time if we're running from bag or sim
    actions.append(
        SetParameter('use_sim_time', True, condition=IfCondition(lu.is_valid(args.rosbag))))
    # # Include the republish_nodes.launch.py
    # actions.append(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(
    #             get_package_share_directory('nvblox_examples_bringup'), 'launch/republish_nodes.launch.py')),
    #         launch_arguments={'use_sim_time': 'true'}.items()
    #     )
    # )

    ## KETI REALSENSE NAV2 ##
    # args.add_arg(
    #     'navigation',
    #     True,
    #     description='Whether to enable nav2 for navigation in Isaac Sim.',
    #     cli=True)

    # # Navigation
    # # NOTE: needs to be called before the component container because it modifies params globally
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/navigation/keti_nvblox_carter_navigation.launch.py',
    #         launch_arguments={
    #             'container_name': NVBLOX_CONTAINER_NAME,
    #             'mode': args.mode,
    #         },
    #         condition=IfCondition(lu.is_true(args.navigation))))

    ## KETI REALSENSE NAV2 ##

    # Realsense
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            # 'launch/sensors/realsense.launch.py',    # keti
            'launch/sensors/keti_compressed_realsense_with_vslam.launch.py',    # keti
            launch_arguments={'container_name': NVBLOX_CONTAINER_NAME},
            condition=UnlessCondition(lu.is_valid(args.rosbag))))

    # # Add the CLI command for republishing the image (raw to compressed)
    # actions.append(
    #     ExecuteProcess(
    #         cmd=[
    #             'ros2', 'run', 'image_transport', 'republish', 'raw', 'compressed',
    #             '--ros-args',
    #             '--remap', 'in:=/camera/color/image_raw_dy',
    #             '--remap', 'out/compressed:=/camera/color/image_raw/compressed_dy',
    #             '--param', 'qos_overrides./camera/color/image_raw.publisher.reliability:=best_effort',
    #             '--param', 'qos_overrides./camera/color/image_raw.subscriber.reliability:=best_effort',
    #             '--param', 'qos_overrides./camera/color/image_raw/compressed_dy.publisher.reliability:=best_effort',
    #             '--param', 'qos_overrides./camera/color/image_raw/compressed_dy.subscriber.reliability:=best_effort'
    #         ],
    #         name='image_republisher_raw_to_compressed_sender',
    #         output='screen'
    #     )
    # )


    # # Add the CLI command for republishing the image (compressed to raw)
    # actions.append(
    #     ExecuteProcess(
    #         cmd=[
    #             'ros2', 'run', 'image_transport', 'republish', 'compressed', 'raw',
    #             '--ros-args',
    #             '--remap', 'in/compressed:=/camera/color/image_raw/compressed_dy',
    #             '--remap', 'out:=/camera/color/image_raw_dy',
    #             '--param', 'qos_overrides./camera/color/image_raw/compressed_dy.publisher.reliability:=reliable',
    #             '--param', 'qos_overrides./camera/color/image_raw/compressed_dy.subscriber.reliability:=reliable',
    #             '--param', 'qos_overrides./camera/color/image_raw_dy.publisher.reliability:=reliable',
    #             '--param', 'qos_overrides./camera/color/image_raw_dy.subscriber.reliability:=reliable'
    #         ],
    #         name='image_republisher_compressed_to_raw',
    #         output='screen'
    #     )
    # )

    # Visual SLAM
    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/vslam.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'camera': NvbloxCamera.realsense,
            },
            # Delay for 1 second to make sure that the static topics from the rosbag are published.
            delay=1.0,
        ))

    # # People segmentation
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/perception/segmentation.launch.py',
    #         launch_arguments={
    #             'container_name': NVBLOX_CONTAINER_NAME,
    #             'people_segmentation': args.people_segmentation,
    #             # 'input_topic': '/camera/color/image_raw',   # keti
    #             'input_topic': '/camera/color/image_raw_dy',   # keti
    #             'input_camera_info_topic': '/camera/color/camera_info', # keti
    #         },
    #         condition=IfCondition(lu.has_substring(args.mode, NvbloxMode.people))))

    # # Nvblox
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/perception/keti_nvblox.launch.py',  # keti
    #         launch_arguments={
    #             'container_name': NVBLOX_CONTAINER_NAME,
    #             'mode': args.mode,
    #             'camera': NvbloxCamera.realsense,
    #         }))

    # # Play ros2bag
    # actions.append(
    #     lu.play_rosbag(
    #         bag_path=args.rosbag,
    #         additional_bag_play_args=args.rosbag_args,
    #         condition=IfCondition(lu.is_valid(args.rosbag))))

    # # Visualization
    # actions.append(
    #     lu.include(
    #         'nvblox_examples_bringup',
    #         'launch/visualization/visualization.launch.py',
    #         launch_arguments={
    #             'mode': args.mode,
    #             'camera': NvbloxCamera.realsense
    #         }))

    # Container
    actions.append(lu.component_container(NVBLOX_CONTAINER_NAME, log_level=args.log_level))

    return LaunchDescription(actions)
