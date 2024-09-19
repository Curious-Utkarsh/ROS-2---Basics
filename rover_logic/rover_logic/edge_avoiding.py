#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class follow_wall_bot(Node):
    def __init__(self):
        super().__init__('follow_wall') ## name of the node
        # publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #subscriber
        self.subscription=self.create_subscription(LaserScan,'/scan',self.get_scan_values,20)
        #periodic publisher call
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        ## Initializing Global values
        ## given a value for VELOCITY
        self.linear_vel = 0.4 
        self.region_1=0;self.region_2=0;self.region_3=0
        self.error=0
        self.case=""
        ## creating a message object to fit new velocities and publish them
        self.velocity=Twist()

    def get_scan_values(self,scan_data):
        ## We have 360 data points so we divide them in 3 region
        ## we say if there is something in the region get the smallest distance value of a point in the area
        
        self.region_1= min(min(scan_data.ranges[0:10])   , 2 )
        self.region_2= min(min(scan_data.ranges[10:20])  , 2 )
        self.region_3= min(min(scan_data.ranges[20:30])  , 2 )
        ## region_3 is Left most , region_2 is middle one and region_1 is right most 
        print(round(self.region_3,3) ,"/",round(self.region_2,3),"/",round(self.region_1,3),"/",self.velocity.angular.z,"/",self.case )
    
    ## helping function
    def control(self):
        self.velocity.linear.x= self.linear_vel
        self.velocity.angular.z=0.0

    ## Publihsing Velocity function call back
    def send_cmd_vel(self):
        if(self.region_1<2): ## last ray block is on the wall
            self.case="basic"
            self.control()## calling helper function
            ## Below are conditions if there is a turn coming ahead
        if(self.region_3 >=2 and self.region_2 >=0.6):
                self.case="BACK"
                self.velocity.linear.x= -0.8 ## increasing speed
                self.velocity.angular.z= -2.2 ## sharp right turn
                time.sleep(0.3)

        ## Publishing Complete values
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=follow_wall_bot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()