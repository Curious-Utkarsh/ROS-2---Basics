#!/usr/bin/env python3
import cv2
import numpy 
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

class line_follower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscriber = self.create_subscription(Image,'/camera/image_raw',self.process_data,10)
        self.bridge = CvBridge() 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity=Twist()
        self.error=0
        self.action=""
        self.kp = 0.01 #Reduces Error Slowly over time, thus preventing jerk.
        self.sharp_turn = False
  
    def send_cmd_vel(self):
        
        if(self.error == None and self.sharp_turn == True):
            if(self.action == "Sharp_Left_Turn"):
                self.velocity.linear.x=2.5
                self.velocity.angular.z= 1.5 #Sharp_Left_Turn
                self.action="Sharp_Left_Turn"
            else:
                self.velocity.linear.x=2.5
                self.velocity.angular.z= -1.5 #Sharp_Right_Turn
                self.action="Sharp_Right_Turn"
        elif(self.error > 0 and self.error < 100 and self.sharp_turn == False):
            self.velocity.linear.x=1.0
            self.velocity.angular.z= self.kp * self.error #Go_Left (+ve error)
            self.action="Go_Left"
            self.get_logger().error(str(self.error))
        elif(self.error < 0 and self.error > -100 and self.sharp_turn == False):
            self.velocity.linear.x=1.0
            self.velocity.angular.z= self.kp * self.error #Go_Right (-ve error)
            self.action="Go_Right"
            self.get_logger().error(str(self.error))


        self.publisher.publish(self.velocity)

    def process_data(self, data): 
        image = self.bridge.imgmsg_to_cv2(data)

        light_line = numpy.array([50,50,50])
        dark_line = numpy.array([200,200,200])
        mask = cv2.inRange(image, light_line,dark_line)

        canny= cv2.Canny(mask,40,10)
        r1=280;c1=0
        width = 640
        height = 110
        img = canny[r1:r1 + height, c1:c1 + width]
        frame_mid = int(width/2)

        edge=[]
        for i in range (width):
            if(img[int(height/2),i]==255):
                edge.append(i)
        print(edge)

        if len(edge) < 2:
            print("Sharp_Turn")
            mid_point = None
        else:
            if len(edge) == 4:
                edge[0] = edge[0]
                edge[1] = edge[2]
            elif len(edge) == 3:
                if edge[1] - edge[0] > 5:  
                    edge[0] = edge[0]
                    edge[1] = edge[1]
                else:  # [193, 194, 507]
                    edge[0] = edge[0]
                    edge[1] = edge[2]

            if len(edge) >= 2 and len(edge) < 5:
                mid_area = (edge[1] - edge[0])
                mid_point = edge[0] + (mid_area / 2)
                img[int(height/2), int(mid_point)] = 255
                print("move_ahead")
            else:
                print("Sharp_Turn")
                mid_point = None

        if mid_point is not None:
            self.error = frame_mid - mid_point
            if(self.sharp_turn == True and abs(self.error) < 100):
                self.sharp_turn = False
        else:
            if(len(edge) == 1 and self.sharp_turn == False):
                self.error = None
                self.sharp_turn = True
                if(edge[0] > frame_mid):
                    self.action = "Sharp_Left_Turn"
                else:
                    self.action = "Sharp_Right_Turn"
    
        # More apparent mid frame pixel
        img[int(height/2), int(frame_mid)] = 255
        img[int(height/2)-1, int(frame_mid)] = 255
        img[int(height/2)+1, int(frame_mid)] = 255
        ## Writing on the Frame as output for better understanding
        f_image = cv2.putText(img, self.action, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,), 2, cv2.LINE_AA)

        cv2.imshow('output image',f_image)
        cv2.waitKey(1)
        

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main(args=None):
  rclpy.init(args=args)
  image_subscriber = line_follower()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()