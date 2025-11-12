from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use4k_arg = DeclareLaunchArgument(
        'use4k',
        default_value='false',
        description='Use 4K resolution (true/false)'
    )
    
    image_quality_arg = DeclareLaunchArgument(
        'image_quality',
        default_value='raw',
        description='Image quality: raw, high, medium, low, tiny'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit',
        default_value='0.0',
        description='FPS limit for image publishing (0.0 = no limit)'
    )
    
    face_width_arg = DeclareLaunchArgument(
        'face_width',
        default_value='512',
        description='Width of each cubemap face in pixels'
    )
    
    face_height_arg = DeclareLaunchArgument(
        'face_height',
        default_value='512',
        description='Height of each cubemap face in pixels'
    )
    
    fov_degrees_arg = DeclareLaunchArgument(
        'fov_degrees',
        default_value='90.0',
        description='Field of view in degrees for each face (default 90, increase for wider view)'
    )
    
    use_lookup_table_arg = DeclareLaunchArgument(
        'use_lookup_table',
        default_value='true',
        description='Use pre-computed lookup tables for better performance'
    )
    
    num_threads_arg = DeclareLaunchArgument(
        'num_threads',
        default_value='0',
        description='Number of threads for parallel processing (0 = auto-detect)'
    )
    
    # Theta driver node
    theta_driver_node = Node(
        package='theta_driver',
        executable='theta_driver_node',
        name='theta_driver',
        output='screen',
        parameters=[{
            'use4k': LaunchConfiguration('use4k'),
            'image_quality': LaunchConfiguration('image_quality'),
            'fps_limit': LaunchConfiguration('fps_limit'),
            'camera_frame': 'camera_link'
        }]
    )
    
    # Optimized cubemap converter node
    cubemap_converter_optimized_node = Node(
        package='theta_driver',
        executable='cubemap_converter_optimized_node',
        name='cubemap_converter_optimized',
        output='screen',
        parameters=[{
            'face_width': LaunchConfiguration('face_width'),
            'face_height': LaunchConfiguration('face_height'),
            'fov_degrees': LaunchConfiguration('fov_degrees'),
            'use_lookup_table': LaunchConfiguration('use_lookup_table'),
            'num_threads': LaunchConfiguration('num_threads'),
            'camera_frame': 'camera_link'
        }]
    )
    
    return LaunchDescription([
        use4k_arg,
        image_quality_arg,
        fps_limit_arg,
        face_width_arg,
        face_height_arg,
        fov_degrees_arg,
        use_lookup_table_arg,
        num_threads_arg,
        theta_driver_node,
        cubemap_converter_optimized_node
    ])
