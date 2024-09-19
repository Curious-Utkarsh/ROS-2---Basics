#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle, GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from my_turtle_bot.turtlebot_controller import TurtleBotControllerNode
import threading
from my_robot_interfaces.action import MoveTurtle
import time
    
class MoveTurtleBotNode(LifecycleNode): 
    def __init__(self):
        super().__init__("move_turtle_bot_3") 
        self.get_logger().info("In Constructor (UnConfigured)")
        self.goal_lock_ = threading.Lock()
        self.goal_handle_: ServerGoalHandle = None
        self.action_server_ = None
        self.server_activate_ = False

    def on_configure(self, previous_state: LifecycleState):
        self.turtle_controller = TurtleBotControllerNode() # Instance of the Class ~ (Object)
        self.action_server_ = ActionServer(self, MoveTurtle, 
                                        "move_turtlebot3", 
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback,
                                        execute_callback=self.execute_callback, 
                                        callback_group=ReentrantCallbackGroup()
                                        )
        self.get_logger().info("MoveTurtleBot Action Server Has Been Configured But is Inactive")
        return TransitionCallbackReturn.SUCCESS 
        
    def on_cleanup(self, previous_state: LifecycleState):
        if self.action_server_ is not None:
            self.get_logger().info("Destroying Action Server...Back to Constructor (Unconfigured) ")
            self.action_server_.destroy()  # Explicitly call destroy on the action server
            self.action_server_ = None
        return TransitionCallbackReturn.SUCCESS 
    
    def on_shutdown(self, previous_state: LifecycleState):
        if self.action_server_ is not None:
            self.get_logger().info("Destroying Action Server...SHUTING DOWN")
            self.action_server_.destroy()  # Explicitly call destroy on the action server
            self.action_server_ = None
        return TransitionCallbackReturn.SUCCESS 
    
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate")
        self.get_logger().info("Action Server is Now Active & Ready to Accept Goal (Active)...")
        self.server_activate_ = True
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("In on_activate")
        self.get_logger().info("Action Server is Deactivated (InActive)...")
        self.server_activate_ = False
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("Aborting Current Goal")
                self.goal_handle_.abort()
        return super().on_deactivate(previous_state)


    def goal_callback(self, goal_request: MoveTurtle.Goal):
        if self.server_activate_ == False:
            self.get_logger().warn("Rejecting the Goal, Server Not Active...")
            return GoalResponse.REJECT

        if goal_request.linear_vel_x < 0.0 or goal_request.linear_vel_x > 3.0:
            self.get_logger().error("Goal Rejected")
            return GoalResponse.REJECT
        
        # Refuse New Goal if current Goal is Active.
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("A goal is already Active, Rejecting new Goal")
                return GoalResponse.REJECT
        
        self.get_logger().info("Goal Accepted")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Recieved a Cancel Request")
        return CancelResponse.ACCEPT
        

    def execute_callback(self, goal_handle: ServerGoalHandle):
        #Getting Request Data from Goal Handle.
        with self.goal_lock_:
            self.goal_handle_ = goal_handle
            linear_x = goal_handle.request.linear_vel_x
            angular_z = goal_handle.request.angular_vel_z
            duration = goal_handle.request.duration_sec

        #Executing Goal
        self.get_logger().info("Executing Goal")
        result = MoveTurtle.Result()
        self.turtle_controller.update_velocity(linear_x, angular_z)
        start_time = time.time()
        while(time.time() - start_time < duration):
            if goal_handle.status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error("Goal was Aborted")
                result.message = "Unable to Reach Goal"
                return result  # Exit the callback without succeeding the goal

            if goal_handle.is_cancel_requested:
                self.get_logger().error("Cancelling Goal")
                goal_handle.canceled()
                result.message = "GOAL CANCELED"
                return result
            
            self.get_logger().info(f"Time Elapsed: {time.time() - start_time:.2f}s, Duration: {duration:.2f}s")
            rclpy.spin_once(self.turtle_controller, timeout_sec=0.1)

        self.get_logger().info("STOPPING now")
        time.sleep(2.0)
        linear_x = -0.1
        angular_z = 0.0
        self.turtle_controller.update_velocity(linear_x, angular_z)
        rclpy.spin_once(self.turtle_controller, timeout_sec=0.1)
        result.message = "REACHED POSTION ON TIME, Now Stopping"
        goal_handle.succeed()
        return result

    
def main(args=None):
    rclpy.init(args=args)
    node1 = MoveTurtleBotNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node1)
    executor.spin()
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
