#!/usr/bin/python3
# -- coding: utf-8 --

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # Find path
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "pcd_viewer.rviz")
    
    # Declare launch arguments
    pcd_file_arg = DeclareLaunchArgument(
        "pcd_file",
        default_value="",
        description="Path to PCD file to visualize. If empty, will try to find PCD files in Log/PCD directory",
    )
    
    topic_name_arg = DeclareLaunchArgument(
        "topic_name",
        default_value="/pcd_map",
        description="Topic name to publish point cloud",
    )
    
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Frame ID for the point cloud",
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate",
        default_value="2.0",
        description="Publishing rate in Hz (0.0 means publish once, higher rate = smoother but more CPU)",
    )
    
    loop_arg = DeclareLaunchArgument(
        "loop",
        default_value="true",
        description="Whether to continuously publish the point cloud",
    )
    
    voxel_size_arg = DeclareLaunchArgument(
        "voxel_size",
        default_value="0.0",
        description="Voxel size for downsampling (0.0 = no downsampling, >0 = downsample for smoother display). Only applies if point cloud > 100k points",
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch Rviz2",
    )

    return LaunchDescription([
        pcd_file_arg,
        topic_name_arg,
        frame_id_arg,
        publish_rate_arg,
        loop_arg,
        voxel_size_arg,
        use_rviz_arg,
        
        # PCD Viewer Node
        Node(
            package="fast_livo",
            executable="pcd_viewer_node",
            name="pcd_viewer_node",
            parameters=[{
                "pcd_file": LaunchConfiguration("pcd_file"),
                "topic_name": LaunchConfiguration("topic_name"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "loop": LaunchConfiguration("loop"),
                "voxel_size": LaunchConfiguration("voxel_size"),
            }],
            output="screen"
        ),
        
        # RViz2
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])

