#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch file để chạy perspective converter với camera_info topic cho calibration
    """
    
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='image_raw',
        description='Input image topic (equirectangular)'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='image_perspective',
        description='Output image topic (perspective)'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera_info',
        description='Camera info topic name'
    )
    
    output_width_arg = DeclareLaunchArgument(
        'output_width',
        default_value='640',
        description='Output image width'
    )
    
    output_height_arg = DeclareLaunchArgument(
        'output_height',
        default_value='480',
        description='Output image height'
    )
    
    fov_degrees_arg = DeclareLaunchArgument(
        'fov_degrees',
        default_value='75.0',
        description='Field of view in degrees'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame name'
    )
    
    distortion_model_arg = DeclareLaunchArgument(
        'distortion_model',
        default_value='plumb_bob',
        description='Camera distortion model (plumb_bob, fisheye, etc.)'
    )
    
    # Perspective converter node
    perspective_converter_node = Node(
        package='theta_driver',
        executable='perspective_converter_node',
        name='perspective_converter',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'output_width': LaunchConfiguration('output_width'),
            'output_height': LaunchConfiguration('output_height'),
            'fov_degrees': LaunchConfiguration('fov_degrees'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'distortion_model': LaunchConfiguration('distortion_model'),
        }],
        output='screen'
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        camera_info_topic_arg,
        output_width_arg,
        output_height_arg,
        fov_degrees_arg,
        camera_frame_arg,
        distortion_model_arg,
        perspective_converter_node,
    ])

