#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed
from functools import partial
    
class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery_client_node") 
        self.counter_ = 0
        self.num_ = 1
        self.state_ = False
        self.timer_ = self.create_timer(1, self.batteryCharge)
        self.get_logger().info("Battery Started ...")

    def call_battery_client(self, led_num, led_status):
        client = self.create_client(SetLed, "led_state_server")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server LED ....")

        request = SetLed.Request()
        request.led_num = led_num
        request.led_status = led_status

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_battery_client, led_num=led_num, led_status=led_status))

    def callback_call_battery_client(self, future, led_num, led_status):
        try:
            response = future.result()
            self.get_logger().info(" Led_Num = " + str(led_num) + " Led_Status = " + str(led_status) + " Response = " + str(response.success))
        except Exception as e:
            self.get_logger().error("Request Failed %r " % (e,))

    def batteryCharge(self):
        self.counter_+= self.num_

        if(self.counter_ > 2 and self.state_ == False):
            self.num_= -1
            self.state_ = True
            self.counter_ = 2
            self.get_logger().warn("Battery Discharging....")

        if(self.counter_ < 0 and self.state_ == True):
            self.num_= 1
            self.state_ = False
            self.counter_ = 0
            self.get_logger().warn("Battery Charging....")

        self.call_battery_client(self.counter_, self.state_)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
