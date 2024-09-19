#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.action import ActionClient
    
    
class CountUntilClientNode(Node): 
    def __init__(self):
        super().__init__("count_until_client") # MODIFY NAME
        self.goal_handle_ = None
        self.count_until_client_ = ActionClient(self, CountUntil, "count_until")
        
    def send_goal(self, target_number, period):
        #Wait for Action Server
        self.count_until_client_.wait_for_server()

        #Create a Goal
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period

        #Send the Goal
        self.get_logger().info("Sending Goal...")
        self.count_until_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

        # Just for Testing : cancel Request 2 sec later, once goal sent
        #self.timer_ = self.create_timer(2.0, self.cancel_goal)

    def cancel_goal(self):
        self.get_logger().info("Sending cancel Request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()


    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal Accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("Goal Rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.reached_number))

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info("Got Feedback: " + str(number))

    
    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClientNode() 
    node.send_goal(6, 1.5)
    # node.send_goal(10, 0.5)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
