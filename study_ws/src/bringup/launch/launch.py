import os

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description(): 

    ld = LaunchDescription()

    n_count = Node(
        package='cpp_pkg',
        executable='NumberCounter'
    )

    n_pub = Node(
        package='cpp_pkg',
        executable='NumberPublisher'
    )

    ld.add_action(n_count)
    ld.add_action(n_pub)

    return ld

