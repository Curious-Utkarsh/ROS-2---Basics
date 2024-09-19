#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import threading
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import CountUntil
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
    
    
class CountUntilServerNode(Node): 
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(self, CountUntil, "count_until", 
                                   goal_callback=self.goal_callback, 
                                   handle_accepted_callback=self.handle_accepted_callback, #<-Just comment this handle_accepted_callback line to run parallel goals.
                                   cancel_callback=self.cancel_callback, 
                                   execute_callback=self.execute_callback, 
                                   callback_group=ReentrantCallbackGroup()) 
        self.get_logger().info("Action Server has been Started...")

    def goal_callback(self, goal_request: CountUntil.Goal):
        self.get_logger().info("Recieved a Goal")

        # # Policy : Refuse New Goal if current Goal is Active. (Comment this for running parallel goals)
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().warn("A goal is already Active, Rejecting new Goal")
        #         return GoalResponse.REJECT

        # Validate the Goal Request
        if goal_request.target_number <= 0:
            self.get_logger().info("Rejecting the Goal")
            return GoalResponse.REJECT
        
        # # Policy : Preempt Current Goal if New Goal Recieved. (Comment this for running parallel goals)
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().warn("Aborting Current Goal & Accepting New Goal")
        #         self.goal_handle_.abort()

        self.get_logger().info("Accepting the Goal")
        return GoalResponse.ACCEPT
    
    def handle_accepted_callback(self, goal_handle: ServerGoalHandle): #to execute in queue
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Recieved a Cancel Request")
        return CancelResponse.ACCEPT # or REJECT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        #Get Request from Goal
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period

        #Execute the action
        self.get_logger().info("Executing the Goal")
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        counter = 0
        for i in range(target_number):
            if not goal_handle.is_active:
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the Goal")
                goal_handle.canceled()
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            counter += 1
            self.get_logger().info(str(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        #Once done set Goal Final State
        # goal_handle.abort()
        goal_handle.succeed()

        #And send the Result
        result.reached_number = counter
        self.process_next_goal_in_queue()
        return result
    
    def process_next_goal_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

    
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode() 
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
