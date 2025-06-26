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
       
        length= len(msg.ranges)
        i =msg.ranges[0]
        
        
        if(i<0.5 ):
            cmd.linear.x =0.0
            cmd.angular.z=1.0
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