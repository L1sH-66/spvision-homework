"""Launch unchanged simulator plus navigation stack; no automatic goal publication."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    def include(package, filename, args=None):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', filename)),
            launch_arguments=(args or {}).items())
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        include('sp_nav_sim', 'sim_robot.launch.py'),
        include('sp_nav_bringup', 'sp_nav.launch.py', {'rviz': LaunchConfiguration('rviz')}),
        Node(package='sp_nav_bringup', executable='evaluate.py', output='screen'),
    ])
