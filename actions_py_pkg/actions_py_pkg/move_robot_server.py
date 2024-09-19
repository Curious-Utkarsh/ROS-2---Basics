#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import threading
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle, GoalStatus
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
    
    
class MoveRobotServerNode(Node): 
    def __init__(self):
        super().__init__("move_robot_server")
        self.current_posn = 50
        self.goal_handle_: ServerGoalHandle = None #class attribute, using this we can access the current executing goal in class and abort it etc.
        self.goal_lock_ = threading.Lock()
        self.move_robot_server_ = ActionServer(self, MoveRobot, "move_robot", 
                                   goal_callback=self.goal_callback, 
                                   cancel_callback=self.cancel_callback, 
                                   execute_callback=self.execute_callback, 
                                   callback_group=ReentrantCallbackGroup()
                                   ) 
        self.get_logger().info("Action Server has been Started...")
        self.get_logger().info("Current Posn.." + str(self.current_posn))

    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Recieved a Goal")

        # Validate the Goal Request
        if goal_request.position_to_go < 0 or goal_request.position_to_go > 100 or goal_request.velocity <= 0:
            self.get_logger().warn("Rejecting the Goal")
            return GoalResponse.REJECT
        
        # New Goal is Valid, Abort the current goal and accept the new goal.
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active: #by this we acces the current goal in execution and abort it
                self.get_logger().warn("Aborting Current Goal & Accepting New Goal")
                self.goal_handle_.abort()

        self.get_logger().info("Accepting the Goal")
        return GoalResponse.ACCEPT 

    def cancel_callback(self, goal_handle: ServerGoalHandle): 
        self.get_logger().info("Recieved a Cancel Request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle): #Goal Handle stores and Manages the data recieved from send_goal from client
        #Get Request from Goal
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
            position_to_go = goal_handle.request.position_to_go
            velocity = goal_handle.request.velocity

        #Execute the action
        self.get_logger().info("Executing the Goal")
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()
    
        while rclpy.ok():

            # Check if the goal has been aborted
            if goal_handle.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("Goal was aborted, by another goal")
                result.reached_position = self.current_posn
                result.status = "GOAL ABORTED"
                return result  # Exit the callback without succeeding the goal
            
            if goal_handle.is_cancel_requested:
                if self.current_posn == position_to_go:
                    self.get_logger().info("Goal Reached not Cancelling")
                    goal_handle.succeed()
                    result.reached_position = self.current_posn
                    result.status = "GOAL SUCCEEDED"
                else:
                    self.get_logger().info("Cancelling Goal")
                    goal_handle.canceled()
                    result.reached_position = self.current_posn
                    result.status = "GOAL CANCELED"
                return result

            diff = position_to_go - self.current_posn
            if(diff == 0):
                result.reached_position = self.current_posn
                result.status = "REACHED SUCCESS"
                goal_handle.succeed()
                return result
                
            elif diff > 0:
                if diff >= velocity:
                    self.current_posn += velocity
                else:
                    self.current_posn += diff
            else:
                if abs(diff) >= velocity:
                    self.current_posn -= velocity
                else:
                    self.current_posn -= abs(diff)


            self.get_logger().info(str(self.current_posn))
            feedback.current_position = self.current_posn
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

    
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
