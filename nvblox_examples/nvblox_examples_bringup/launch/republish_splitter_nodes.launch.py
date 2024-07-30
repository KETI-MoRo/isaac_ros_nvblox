from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republisher_compressed_to_raw_color',
        #     output='screen',
        #     arguments=[
        #         'compressed', 'raw',
        #         '--ros-args',
        #         '--remap', 'in/compressed:=/camera/color/image_raw/compressed',
        #         '--remap', 'out:=/camera/color/image_raw'
        #     ]
        # ),

        ## Depth 는 republish가 안 된다.
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republisher_compressed_to_raw_depth',
        #     output='screen',
        #     arguments=[
        #         'compressed', 'raw',
        #         '--ros-args',
        #         '--remap', 'in/compressed:=/camera/depth/image_rect_raw/compressedDepth',
        #         '--remap', 'out:=/camera/depth/image_rect_raw'
        #     ]
        # ),
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republisher_compressed_to_raw_infra1',
        #     output='screen',
        #     arguments=[
        #         'compressed', 'raw',
        #         '--ros-args',
        #         '--remap', 'in/compressed:=/camera/infra1/image_rect_raw/compressed',
        #         '--remap', 'out:=/camera/infra1/image_rect_raw'
        #     ]
        # ),
        # Node(
        #     package='image_transport',
        #     executable='republish',
        #     name='image_republisher_compressed_to_raw_infra2',
        #     output='screen',
        #     arguments=[
        #         'compressed', 'raw',
        #         '--ros-args',
        #         '--remap', 'in/compressed:=/camera/infra2/image_rect_raw/compressed',
        #         '--remap', 'out:=/camera/infra2/image_rect_raw'
        #     ]
        # ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher_raw_to_compressed_depth',
            output='screen',
            arguments=[
                'raw', 'compressedDepth',
                '--ros-args',
                '--remap', 'in:=/camera/realsense_splitter_node/output/depth',
                '--remap', 'out/compressedDepth:=/camera/realsense_splitter_node/output/depth/compressed'
            ],
            #parameters=[camera_params]
        )
    ])
