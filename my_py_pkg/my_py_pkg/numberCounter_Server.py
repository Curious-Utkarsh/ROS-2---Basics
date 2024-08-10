#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from std_srvs.srv import SetBool
    
    
class NumberCounterServerNode(Node): 
    def __init__(self):
        super().__init__("number_counter") 
        self.counter_ = 0
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.subscriber_ = self.create_subscription(Int64, "number", self.timerback, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Counter Started")

    def timerback(self, msg):
        self.counter_+=msg.data
        new_msg = Int64()
        new_msg.data = int(self.counter_)
        self.publisher_.publish(new_msg)
        self.get_logger().info(str(new_msg.data))

    def callback_reset_counter(self, request, response):
        if(request.data == True):
            self.counter_= 0
            response.success = True
            response.message = "Reset Successful"
        else:
            response.success = False
            response.message = "Reset Unsuccessful"
        return response
        
        
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterServerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
