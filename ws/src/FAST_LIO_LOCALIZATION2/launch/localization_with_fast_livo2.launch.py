#!/usr/bin/python3
"""
Launch file để chạy Localization2 cùng với FAST-LIVO2
Localization2 sẽ sử dụng map từ FAST-LIVO2 và cung cấp localization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    fast_livo_pkg = get_package_share_directory("fast_livo")
    localization_pkg = get_package_share_directory("fast_lio_localization")
    
    # Default config paths
    fast_livo_config = os.path.join(fast_livo_pkg, "config", "mid360_equirectangular.yaml")
    localization_config_path = os.path.join(localization_pkg, "config")
    default_rviz_config = os.path.join(localization_pkg, "rviz", "fastlio_localization.rviz")
    
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    fast_livo_config_file = LaunchConfiguration("fast_livo_config")
    localization_config_file = LaunchConfiguration("localization_config")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    map_root = LaunchConfiguration("map_root")
    use_fast_livo = LaunchConfiguration("use_fast_livo")
    
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="false", 
        description="Use simulation (Gazebo) clock if true"
    )
    
    declare_fast_livo_config = DeclareLaunchArgument(
        "fast_livo_config",
        default_value=fast_livo_config,
        description="FAST-LIVO2 config file path"
    )
    
    declare_localization_config = DeclareLaunchArgument(
        "localization_config",
        default_value="mid360.yaml",
        description="Localization2 config file name"
    )
    
    declare_rviz = DeclareLaunchArgument(
        "rviz", 
        default_value="true", 
        description="Use RViz to monitor results"
    )
    
    declare_rviz_cfg = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=default_rviz_config,
        description="RViz config file path"
    )
    
    declare_map_root = DeclareLaunchArgument(
        "map_root", 
        default_value="", 
        description="Directory containing pose.json and pcd tiles for localization"
    )
    
    declare_use_fast_livo = DeclareLaunchArgument(
        "use_fast_livo",
        default_value="true",
        description="Run FAST-LIVO2 mapping node (set to false if running separately)"
    )
    
    # FAST-LIVO2 mapping node (optional, can be disabled if running separately)
    fast_livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="fast_livo_mapping",
        parameters=[fast_livo_config_file],
        condition=IfCondition(use_fast_livo),
        output="screen"
    )
    
    # Localization2 node
    localization_node = Node(
        package="fast_lio_localization",
        executable="fastlio_mapping",
        name="fast_lio_localization",
        parameters=[
            PathJoinSubstitution([localization_config_path, localization_config_file]),
            {
                "use_sim_time": use_sim_time,
                "map_root": map_root,
            },
        ],
        output="screen",
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", rviz_cfg], 
        condition=IfCondition(rviz_use)
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_fast_livo_config,
        declare_localization_config,
        declare_rviz,
        declare_rviz_cfg,
        declare_map_root,
        declare_use_fast_livo,
        fast_livo_node,
        localization_node,
        rviz_node,
    ])

