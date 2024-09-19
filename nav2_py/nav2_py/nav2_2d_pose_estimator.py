#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
    
    
class PoseEstimatorNode(Node): 
    def __init__(self):
        super().__init__("pose_estimator") 
        self.nav = BasicNavigator()
        self.declare_parameter("set_pose", False)
        self.setPose = self.get_parameter("set_pose").value
        self.set_pose_and_goal()
        
    def set_pose_and_goal(self):
        if (self.setPose == True): #Ensure 2d pose is set only once, then you use code only to send goals.
            # Set Initial Pose (2D Pose Estimator )
            q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0) #Initial Posn of robot 0,0,0 converts to quaternion value (x,y) always remain 0 here,dont change them #if robot faces to left wrt to x-axis(+ve) give z as 1.57(radian) ie 90deg left vice versa for rhight -1.57 [right hand thumb rule], here initially robot is spawned facing forward so we give z as 0
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0 #Robot Posn. wrt Map tf, which is the fixed link.
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.position.z = 0.0 
            initial_pose.pose.orientation.x = q_x
            initial_pose.pose.orientation.y = q_y
            initial_pose.pose.orientation.z = q_z
            initial_pose.pose.orientation.w = q_w
            self.nav.setInitialPose(initial_pose)

        #Wait for Nav2
        self.nav.waitUntilNav2Active()

        #Send Nav2 Goal
        goal_pose = PoseStamped()
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57) #(x,y) always remain 0 ,dont change them #you want robot to face left at end of goal so give z as 1.57 here, turn 90deg left at end of goal.
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 1.5 #Robot Posn. to go wrt to map tf (fixed)
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.position.z = 0.0 #always 0, cause bot cant fly
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        self.nav.goToPose(goal_pose)

        while not self.nav.isTaskComplete(): #Waits for bot to reach the goal
            # feedback = self.nav.getFeedback()
            # print(feedback)
            pass

        print(self.nav.getResult()) #Succeed of Aborted etc (action server nav2 is)
    
    
def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode() 
    rclpy.spin_once(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
