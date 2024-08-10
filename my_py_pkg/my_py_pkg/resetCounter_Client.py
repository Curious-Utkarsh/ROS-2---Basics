#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from functools import partial
    
class ResetCounterNode(Node): 
    def __init__(self):
        super().__init__("reset_counter") 
        self.call_reset_counter_client(True)

    def call_reset_counter_client(self, data):
        client = self.create_client(SetBool, "reset_counter")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server to Reset Counter ....")

        request = SetBool.Request()
        request.data = data

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_reset_counter_client, data=data))

    def callback_call_reset_counter_client(self, future, data):
        try:
            response = future.result()
            self.get_logger().info(str(response.success) + "  " + str(response.message))
        except Exception as e:
            self.get_logger().error("Request Failed %r " % (e,))
    
def main(args=None):
    rclpy.init(args=args)
    node = ResetCounterNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
