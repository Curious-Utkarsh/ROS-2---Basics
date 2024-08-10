#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus
    
    
class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("hw_status_publisher") 
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1, self.timercallback)
        self.get_logger().info("Node Started...")

    def timercallback(self):
        msg = HardwareStatus()
        msg.temperature = 45
        msg.are_motors_ready = True
        msg.debug_message = "Nothing Special..."
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
