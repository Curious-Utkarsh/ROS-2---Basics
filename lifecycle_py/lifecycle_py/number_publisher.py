#!/usr/bin/env python3
import rclpy
#from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.get_logger().info("In Constructor")
        self.number_ = 1
        self.publish_frequency_ = 1.0
        self.number_publisher_ = None
        self.number_timer_ = None

    #Create ROS2 Commun., connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("In on_configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10)
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel() # you create the timer and cancel, because you want the timer to start from 1, when you activate, else it will start here only, but will not display.
        return TransitionCallbackReturn.SUCCESS # or FAILURE
    
    #Activate / Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate")
        self.number_timer_.reset() # timer starts from 1, or from where you deactivated and left, else it will keep on running but will not show.
        return super().on_activate(previous_state)
    
    #Deactivate / Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state)
    
    #Destroy ROS2 Commun., Disconnect HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("In on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS # or FAILURE
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("In on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS # or FAILURE
        

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1
        

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()