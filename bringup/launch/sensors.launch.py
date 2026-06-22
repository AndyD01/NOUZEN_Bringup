from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_dir = FindPackageShare('nouzen_bringup')

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'bringup', 'launch', 'include', 'ld19.launch.py'])]),
    )

    launch_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                bringup_dir, 'bringup', 'launch', 'include', 'camera.launch.py'])]),
    )

    ld = LaunchDescription()
    ld.add_action(launch_lidar)
    ld.add_action(launch_camera)
    return ld