#CALL IT moverobot_startup.py instaed of the above name, from next time
#If we have Two Action Servers, then we require two clinets also to manage their transition, as trasition is manged by services, so two diff services so tweo diff clients
# service name is nodename/change_stae, so twop diff node nbnames so two diff clients for 2 services of two actoion servers.

#This focuses on once all the nodes are configured then only activate all nodes, if say out of 3 one client fails to configure, then retry that, till all are config. then only activate all nodes.

#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class MoveRobotLifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("move_robot_lifecycle_manager")
        self.declare_parameter("move_robot_lifecycle_node_names", rclpy.Parameter.Type.STRING_ARRAY)
        node_name_list_ = self.get_parameter("move_robot_lifecycle_node_names").value
        self.get_logger().info("Node Names List:" + str(node_name_list_))
        self.client_list = [] #Multiple Clients at once Concept, Important.
        for node_name in node_name_list_:
            service_change_state_name = "/" + node_name + "/change_state"
            self.client_list.append(self.create_client(ChangeState, service_change_state_name))
        
    def change_state(self, transition: Transition): #Handling Multiple clients
        for client in self.client_list:
            client.wait_for_service()
            request = ChangeState.Request()
            request.transition = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
    
    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Configuring OK, now inactive")

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Activating OK, now active")


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotLifecycleNodeManager()
    node.initialization_sequence() #Here we dont want to spin the node, just run the script once and thats all.
    rclpy.shutdown()


if __name__ == "__main__":
    main()