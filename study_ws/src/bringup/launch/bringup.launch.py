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

    remap_number_topic=('number', 'G_NUM')
    remap_counter_topic=('number_count', 'CTR')

    n_count = Node(
        package='cpp_pkg',
        executable='NumberCounter',
        name='N_CTR',
        remappings=[remap_number_topic]
    )

    n_pub = Node(
        package='cpp_pkg',
        executable='NumberPublisher',
        name='N_PUB',
        remappings=[remap_number_topic, remap_counter_topic]
    )

    ld.add_action(n_count)
    ld.add_action(n_pub)

    return ld

