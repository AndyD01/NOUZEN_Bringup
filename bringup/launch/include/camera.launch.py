from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext

def generate_camera_launch(context: LaunchContext):
    # Construim calea absolută corectă către fișierul YAML
    pkg_share = FindPackageShare('nouzen_bringup').perform(context)
    
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'pixel_format': 'YUYV',
            'time_per_frame': [1, 15],
            'camera_frame_id': 'camera_link_optical',
            'camera_info_url': 'file:///home/saim/saim_nouzen/src/nouzen_bringup/bringup/config/camera_info.yaml',
            'io_method': 'mmap',
        }]
    )
    return [camera_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=generate_camera_launch)
    ])