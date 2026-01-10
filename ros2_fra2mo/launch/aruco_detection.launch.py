from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch argument for marker ID
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='1',
        description='ID of the ArUco marker to detect (1, 2, 3, 4, 5, 6)'
    )
    
    # Get the marker_id parameter
    marker_id = LaunchConfiguration('marker_id')
    
    # ArUco detection node
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.2,  # 20cm marker size
            'marker_id': marker_id,
            'reference_frame': 'camera_optical_link',
            'camera_frame': 'camera_optical_link',
            'marker_frame': 'aruco_marker_frame',
            'corner_refinement': 'LINES',
        }],
        remappings=[
            ('/camera_info', '/fra2mo/camera_info'),
            ('/image', '/fra2mo/camera'),
        ]
    )
    
    return LaunchDescription([
        marker_id_arg,
        aruco_node
    ])
