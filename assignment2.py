#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class turtle(Node):
    def __init__(self):
        super().__init__("assignment2")
        self.suscriber = self.create_subscription(LaserScan ,"/scan",self.pub,10)
        self.publish1 = self.create_publisher(Twist ,"/cmd_vel",10)
        

    def pub(self ,msg = LaserScan):
        cmd=Twist()
       
        # print(len(msg.ranges)) used earlies to check the range of scan and what values of degrees were stored at what index 
        # print(msg.ranges[0])    
        # print(msg.ranges[180])
        # print(msg.ranges[359])

        i =msg.ranges[0]     #scans for obstacle in the area ahead upto +- 15 degrees 
        j =msg.ranges[15]
        k =msg.ranges[345]
        
        if((i< 0.5) or(j< 0.6) or(k<0.6)):
            cmd.linear.x =0.0
            cmd.angular.z= -0.75  #turns right 
        else:
            cmd.linear.x =0.5
            cmd.angular.z =0.0   
        self.publish1.publish(cmd)


def main (args=None):
    rclpy.init(args=args)
    
    node = turtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
   main()  