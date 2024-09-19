#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.action import MoveRobot
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.action import ActionClient
from example_interfaces.msg import Empty
    
    
class MoveRobotClientNode(Node): 
    def __init__(self):
        super().__init__("move_robot_client") 
        self.goal_handle_ = None
        self.subscriber_ = self.create_subscription(Empty, "cancel_move", self.callback_cancel_move, 10)
        self.move_robot_client_ = ActionClient(self, MoveRobot, "move_robot")
        
    def send_goal(self, position_to_go, velocity):
        #Wait for Action Server
        self.move_robot_client_.wait_for_server()

        #Create a Goal
        goal = MoveRobot.Goal()
        goal.position_to_go = position_to_go
        goal.velocity = velocity

        #Send the Goal
        self.get_logger().info("Sending Goal...")
        self.move_robot_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        if self.goal_handle_ is not None:
            self.get_logger().info("Sending cancel Request")
            self.goal_handle_.cancel_goal_async()


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
            self.get_logger().info("Succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info("Result: " + str(result.reached_position) + "  " + str(result.status))


    def goal_feedback_callback(self, feedback_msg):
        current_posn = feedback_msg.feedback.current_position
        self.get_logger().info("FEEDBACK POSN : " + str(current_posn))

    def callback_cancel_move(self, msg):
        self.cancel_goal()

    
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotClientNode() 
    node.send_goal(76, 1)
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
