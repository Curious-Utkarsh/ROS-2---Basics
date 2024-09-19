#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('obstacle_avoid') ## name of the node
        # publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #subscriber
        self.subscription=self.create_subscription(LaserScan,'/scan',self.get_scan_values, 10)
        #periodic publisher call
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        ## Initializing Global values
        ## given a value for VELOCITY
        self.linear_vel = 0.22 
        ## Making dictionary to divide the area of laser scan 
        self.regions={'right': 0,'mid': 0,'left': 0}
        ## creating a message object to fit new velocities and publish them
        self.velocity=Twist()
        self.kp = 0.6


    ## Subscriber Callback function 
    def get_scan_values(self,scan_data):
        ## We have 360 data points and we are dividing them in 3 regions
        ## we say if there is something in the region get the smallest value
        self.regions = {
        'right':   int(min(min(scan_data.ranges[0:120])  , 5)),
        'mid':     int(min(min(scan_data.ranges[120:240]), 5)),
        'left':    int(min(min(scan_data.ranges[240:360]), 5)),
        }
        print(self.regions['left']," / ",self.regions['mid']," / ",self.regions['right'])

  
    ## Callback Publisher of velocities called every 0.2 seconds
    def send_cmd_vel(self):
        ## angular and linear velocities are set into object self.velcity
        ## setting the linear velocity to be fixed and robot will keep on moving
        self.velocity.linear.x=self.linear_vel
        ## cases to make the robot change its angular velocity
        if(self.regions['left'] > 4  and self.regions['mid'] > 4   and self.regions['right'] > 4 ):
            self.velocity.angular.z=0.0 # condition in which area is total clear
            print("forward")
        elif(self.regions['left'] > 4 and self.regions['mid'] > 4  and self.regions['right'] < 4 ):
            error = (3.5 - self.regions['right'])
            self.velocity.angular.z=1.57*error# object on right,taking  left 
            print("left")
        elif(self.regions['left'] < 4 and self.regions['mid'] > 4  and self.regions['right'] > 4 ):
            error = (3.5 - self.regions['left'])
            self.velocity.angular.z=-1.57*error #object  on left, taking  right
            print("left")          
        elif(self.regions['left'] < 4 and self.regions['mid'] < 4  and self.regions['right'] < 4 ):
            self.velocity.linear.x=-self.linear_vel
            self.velocity.angular.z=3.14# object ahead take full turn
            print("reverse")
        elif(self.regions['mid'] < 4  and (self.regions['right'] > 4 or self.regions['left'] > 4)):
            if(self.regions['right'] > self.regions['left']):
                error = (3.5 - self.regions['left'])
                self.velocity.angular.z=-1.57*error #object  on left, taking  right
                print("right")  
            else:
                error = (3.5 - self.regions['right'])
                self.velocity.angular.z=1.57*error  

        else:## lThis code is not completed ->  you have  to add more conditions  ot make it robust
            print("some other conditions are required to be programmed") 
       
        ## lets publish the complete velocity
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()