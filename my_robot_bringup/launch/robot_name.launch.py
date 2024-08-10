from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_names = ["Robot_1", "Robot_2", "Robot_3"]

    robot_station_nodes = []

    for name in robot_names:
        robot_station_nodes.append(Node(
            package = "my_py_pkg",
            executable = "robot_news_station",
            name = "robot_station_node_" + name.lower(),
            parameters = [
                {"robot_name": name}
            ]
        ))

    smartphone = Node(
        package = "my_cpp_pkg",
        executable = "smartphone",
        name = "smartphone",
    )

    for node in robot_station_nodes:
        ld.add_action(node)
    ld.add_action(smartphone)

    return ld