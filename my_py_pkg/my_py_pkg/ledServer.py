#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStateArray
    
class ledServerNode(Node): 
    def __init__(self):
        super().__init__("led_state_server") 
        self.declare_parameter("led_states", [0, 0, 0])
        self.ledPanel = self.get_parameter("led_states").value
        self.server_ = self.create_service(SetLed, "led_state_server", self.callback_led_state_server)
        self.publisher_ = self.create_publisher(LedStateArray, "led_panel_state", 10)
        self.timer_ = self.create_timer(1, self.ledStatePub)
        self.get_logger().info("LED STATE Server has Started")

    def callback_led_state_server(self, request, response):
        led_num = request.led_num
        led_status = request.led_status
        if(led_num == 0 and led_status == True):
            self.ledPanel = [1, 0, 0]
            response.success = True
        elif(led_num == 1 and led_status == True):
            self.ledPanel = [1, 1, 0]
            response.success = True
        elif(led_num == 2 and led_status == True):
            self.ledPanel = [1, 1, 1]
            response.success = True
        elif(led_num == 2 and led_status == False):
            self.ledPanel = [1, 1, 0]
            response.success = True
        elif(led_num == 1 and led_status == False):
            self.ledPanel = [1, 0, 0]
            response.success = True
        elif(led_num == 0 and led_status == False):
            self.ledPanel = [0, 0, 0]
            response.success = True
        else:
            response.success = False
        return response
        
    def ledStatePub (self):
        msg = LedStateArray()
        msg.led_states = self.ledPanel
        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = ledServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
