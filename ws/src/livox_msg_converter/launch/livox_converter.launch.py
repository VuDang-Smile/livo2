#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='Input topic name for CustomMsg (default: /livox/lidar from driver with remap)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/livox/points',
        description='Output topic name for PointCloud2'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='livox_frame',
        description='Frame ID for output PointCloud2'
    )

    relay_enable_arg = DeclareLaunchArgument(
        'enable_relay',
        default_value='true',
        description='Enable relay node to republish CustomMsg to /livox/lidar'
    )

    # Converter node
    converter_node = Node(
        package='livox_msg_converter',
        executable='livox_msg_converter_node',
        name='livox_msg_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen'
    )

    # Relay node (republish CustomMsg to /livox/lidar)
    relay_node = Node(
        package='livox_msg_converter',
        executable='livox_msg_relay_node',
        name='livox_msg_relay',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/livox/lidar',
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_relay'))
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        frame_id_arg,
        relay_enable_arg,
        converter_node,
        relay_node,
    ])

