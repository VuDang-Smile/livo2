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
    
    # Cubemap converter node
    cubemap_converter_node = Node(
        package='theta_driver',
        executable='cubemap_converter_node',
        name='cubemap_converter',
        output='screen',
        parameters=[{
            'face_width': LaunchConfiguration('face_width'),
            'face_height': LaunchConfiguration('face_height'),
            'fov_degrees': LaunchConfiguration('fov_degrees'),
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
        theta_driver_node,
        cubemap_converter_node
    ])

