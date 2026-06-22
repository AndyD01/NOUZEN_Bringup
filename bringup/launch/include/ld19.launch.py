from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld19_params = PathJoinSubstitution([
        FindPackageShare('nouzen_bringup'),
        'bringup',
        'config',
        'ld19.yaml'
    ])

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        parameters=[ld19_params],
        output='screen'
    )

    return LaunchDescription([
        ldlidar_node,
    ])
