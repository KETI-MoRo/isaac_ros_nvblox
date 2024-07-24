from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher_compressed_to_raw_color',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('in/compressed', '/camera/color/image_raw/compressed'),
                ('out', '/camera/color/image_raw')
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher_compressed_to_raw_depth',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('in/compressed', '/camera/depth/image_rect_raw/compressedDepth'),
                ('out', '/camera/depth/image_rect_raw')
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher_compressed_to_raw_infra1',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('in/compressed', '/camera/infra1/image_rect_raw/compressedDepth'),
                ('out', '/camera/infra1/image_rect_raw')
            ]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republisher_compressed_to_raw_infra2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('in/compressed', '/camera/infra2/image_rect_raw/compressedDepth'),
                ('out', '/camera/infra2/image_rect_raw')
            ]
        )
    ])
