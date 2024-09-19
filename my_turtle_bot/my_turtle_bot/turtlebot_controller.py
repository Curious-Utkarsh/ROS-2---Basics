#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy.parameter
    
    
class TurtleBotControllerNode(Node): 
    def __init__(self):
        super().__init__("turtlebot_controller")
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer_ = self.create_timer(1.0, self.turtlebot_publisher)
        
    def turtlebot_publisher(self):
        self.linear_x += 0.1
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        
    def update_velocity(self, linear_x, angular_z):
        self.linear_x = linear_x
        self.angular_z = angular_z

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()



