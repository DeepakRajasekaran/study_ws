from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtle_sim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )

    deity_node = Node(
        package="turtle_hunter_cpp",
        executable="deity_",
        parameters=[
            {"spawn_freq": 1.0}
        ]
    )

    hunter_node = Node(
        package="turtle_hunter_cpp",
        executable="hunter_",
        parameters=[
            {"kill_closest_turtle_first": True},
            {"hunter_freq": 80}
        ]
    )

    ld.add_action(turtle_sim_node)
    ld.add_action(hunter_node)
    ld.add_action(deity_node)

    return ld