#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy.parameter
    
    
class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller")
        self.linear_x = 0.0
        self.angular_z = 1.0
        self.declare_parameter("turtle_name", "TurtleBot3")
        self.topic_name = self.get_parameter("turtle_name").value
        self.publisher_ = self.create_publisher(Twist, "/"+str(self.topic_name)+"/cmd_vel", 10)
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
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
