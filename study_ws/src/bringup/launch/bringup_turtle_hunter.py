import os
from launch import launch_description
from launch_ros.actions import Node

def generate_launch_description():
    ld = launch_description()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim_node"
    )

    hunter_node = Node(
        package="turtle_hunter", 
        executable="hunter",
        name="hunter",
        parameters=[{
            "kill_closest_turtle_first": True,  
            "hunter_freq": 80  
        }]
    )

    # Node for the deity
    deity_node = Node(
        package="turtle_hunter",
        executable="deity",
        name="deity",
        parameters=[{
            "spawn_freq": 1.0
        }] 
    )

    ld.add_action(turtlesim_node)
    ld.add_action(hunter_node)
    ld.add_action(deity_node)

    return ld
