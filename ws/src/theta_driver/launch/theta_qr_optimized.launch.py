#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('theta_driver')
    
    # Declare launch arguments
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame name'
    )
    
    face_width_arg = DeclareLaunchArgument(
        'face_width',
        default_value='768',
        description='Face width for QR detection (larger = better detection)'
    )
    
    face_height_arg = DeclareLaunchArgument(
        'face_height',
        default_value='768',
        description='Face height for QR detection (larger = better detection)'
    )
    
    enable_qr_detection_arg = DeclareLaunchArgument(
        'enable_qr_detection',
        default_value='true',
        description='Enable QR code detection'
    )
    
    min_confidence_arg = DeclareLaunchArgument(
        'min_confidence',
        default_value='0.3',
        description='Minimum confidence for QR detection (lower = more sensitive)'
    )
    
    detection_interval_arg = DeclareLaunchArgument(
        'detection_interval',
        default_value='1',
        description='Process every N frames (1 = every frame for best detection)'
    )
    
    enable_preprocessing_arg = DeclareLaunchArgument(
        'enable_preprocessing',
        default_value='true',
        description='Enable histogram equalization and contrast enhancement (better detection)'
    )
    
    enable_multithreading_arg = DeclareLaunchArgument(
        'enable_multithreading',
        default_value='true',
        description='Enable parallel face processing'
    )
    
    max_faces_per_thread_arg = DeclareLaunchArgument(
        'max_faces_per_thread',
        default_value='3',
        description='Maximum faces per thread'
    )
    
    enable_rotation_detection_arg = DeclareLaunchArgument(
        'enable_rotation_detection',
        default_value='true',
        description='Enable rotation detection for top/bottom faces'
    )
    
    qr_scale_factor_arg = DeclareLaunchArgument(
        'qr_scale_factor',
        default_value='2',
        description='Scale factor for QR detection (higher = better detection)'
    )
    
    prioritize_top_bottom_arg = DeclareLaunchArgument(
        'prioritize_top_bottom',
        default_value='true',
        description='Prioritize top and bottom faces for detection'
    )
    
    # QR Detector Optimized Node
    qr_detector_optimized_node = Node(
        package='theta_driver',
        executable='qr_detector_optimized_node',
        name='qr_detector_optimized',
        output='screen',
        parameters=[{
            'camera_frame': LaunchConfiguration('camera_frame'),
            'face_width': LaunchConfiguration('face_width'),
            'face_height': LaunchConfiguration('face_height'),
            'enable_qr_detection': LaunchConfiguration('enable_qr_detection'),
            'min_confidence': LaunchConfiguration('min_confidence'),
            'detection_interval': LaunchConfiguration('detection_interval'),
            'enable_preprocessing': LaunchConfiguration('enable_preprocessing'),
            'enable_multithreading': LaunchConfiguration('enable_multithreading'),
            'max_faces_per_thread': LaunchConfiguration('max_faces_per_thread'),
            'enable_rotation_detection': LaunchConfiguration('enable_rotation_detection'),
            'qr_scale_factor': LaunchConfiguration('qr_scale_factor'),
            'prioritize_top_bottom': LaunchConfiguration('prioritize_top_bottom'),
        }],
        remappings=[
            ('image_faces', 'image_faces_optimized'),  # Subscribe to optimized cubemap
        ]
    )
    
    return LaunchDescription([
        camera_frame_arg,
        face_width_arg,
        face_height_arg,
        enable_qr_detection_arg,
        min_confidence_arg,
        detection_interval_arg,
        enable_preprocessing_arg,
        enable_multithreading_arg,
        max_faces_per_thread_arg,
        enable_rotation_detection_arg,
        qr_scale_factor_arg,
        prioritize_top_bottom_arg,
        qr_detector_optimized_node,
    ])
