from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    joy_params = PathJoinSubstitution([
        FindPackageShare('nouzen_bringup'),
        'bringup',
        'config',
        'teleop_xbox.yaml'
    ])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 0.0,
        }],
        output='screen'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),  # TwistStamped topic
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
    ])
