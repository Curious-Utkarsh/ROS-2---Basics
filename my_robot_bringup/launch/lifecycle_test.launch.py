from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    ld = LaunchDescription()

    number_node_name = "my_number_publisher"

    # its a lifecycle node
    number_node = LifecycleNode(
        package = "lifecycle_py",
        executable = "number_publisher",
        name = "my_number_publisher", #its mandatorty to give name in lifecycle nodes
        namespace = "" #Mandatory
    )

    # its a normal node
    lifecycle_node_manager = Node(
        package = "lifecycle_py",
        executable = "lifecycle_node_manager",
        parameters = [
            {"lifecycle_node_name": number_node_name}
        ]
    )

    ld.add_action(number_node)
    ld.add_action(lifecycle_node_manager)
    return ld