#!/usr/bin/env python3
"""
Launch file cho Livox Message Converter Node
Convert CustomMsg từ /livox/lidar sang PointCloud2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='Input topic name (CustomMsg)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/livox/points2',
        description='Output topic name (PointCloud2). Mặc định dùng /livox/points2 để tránh conflict'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='Frame ID cho PointCloud2 message'
    )
    
    use_points2_arg = DeclareLaunchArgument(
        'use_points2_if_exists',
        default_value='true',
        description='Nếu true, sẽ dùng /livox/points2 nếu /livox/points đã tồn tại'
    )

    # Converter node
    converter_node = Node(
        package='livox_msg_converter',
        executable='livox_msg_converter_node',
        name='livox_msg_converter',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_points2_if_exists': LaunchConfiguration('use_points2_if_exists'),
        }]
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        frame_id_arg,
        use_points2_arg,
        converter_node,
    ])

