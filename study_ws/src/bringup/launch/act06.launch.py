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

    robot_names_ = ['R2D2', 'Chappie', 'BayMax', 'Alita', 'Tars']

    station_nodes_ = []

    for name in robot_names_:
        station_nodes_.append(
                Node(
                        package='cpp_pkg',
                        executable='news_station',
                        name='Station_Host_' + name.lower(),
                        parameters=[
                                {'publishFrequency':0.5},
                                {'robot_name_':name}
                            ]
                        )
                    )
    station_nodes_.append(
         Node(
              package='cpp_pkg',
              executable='radio',
              name='radio'
         )
    )

    for nodee in station_nodes_:
            ld.add_action(nodee)

    return ld