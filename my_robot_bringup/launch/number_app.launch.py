from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rename_number_topic = ("number", "new_number_topic")

    number_publisher_node = Node(
        package = "my_py_pkg",
        executable = "number_publisher",
        name = "renamed_num_pub",
        remappings=[rename_number_topic],
        parameters = [
            {"number_to_publish": 4},
            {"publish_timePeriod": 0.1}
        ]
        
    )
# if you want to rename a service then same as topic, first old name then new service name,
# in the remappings bracket where topic is present beside that.
    number_counter_node = Node(
        # package = "my_cpp_pkg",
        package = "my_py_pkg",
        executable = "number_counter",
        name = "rename_num_counter",
        remappings=[rename_number_topic,
                    ("number_count", "new_number_counter_topic")] 
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    return ld