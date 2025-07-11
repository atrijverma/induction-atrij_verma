#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import time 


class aruco_nav(Node):
    def __init__(self):
        super().__init__('aruco_nav')
        self.bridge = CvBridge()
        self.vel_pub = self.create_publisher(Twist ,'/cmd_vel', 10)
        self.camera = self.create_subscription(Image,'/camera/image_raw',self.camera_callback, 10)
        
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters()

        self.detected_count = 0
        self.turning = False

    def camera_callback(self,msg):
        
        cmd1 =Twist()
        cmd1.linear.x =0.25
        self.vel_pub.publish(cmd1)

        if self.detected_count >= 5:
            cmd1.linear.x =0.0
            self.vel_pub.publish(cmd1)
            return

        if(self.turning == False):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is None:
                pass
            else:
                self.get_logger().info(f"Detected IDs: {ids.flatten().tolist()}")

        

            marker_length = 0.1778  

        
            camera_matrix = np.array([[1696.80268, 0.0, 960.5],
                                    [0.0, 1696.80268 , 540.5],
                                    [0.0, 0.0, 1.0]])
            dist_coeffs = np.zeros((5, 1))  

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs)
            if tvecs is not None and len(tvecs) > 0:
                distance_to_tag = tvecs[0][0][2]  
                self.get_logger().info(f"Distance to tag {ids[0][0]}: {distance_to_tag:.2f} m")
            else:
                distance_to_tag = 0.0
        
            if (distance_to_tag < 3.5):
                duration1 = (distance_to_tag -0.3)/0.25
                if duration1 >0.0 :
                    start_time =self.get_clock().now().nanoseconds / 1e9
                    while self.get_clock().now().nanoseconds / 1e9 -start_time <(duration1 +0.1):
                        self.vel_pub.publish(cmd1)


                if ids is not None and len(ids) > 0:
                    ids = ids.flatten()
                    for id in ids:
                        if id == 0:
                            cmd1.linear.x =0.0
                            self.vel_pub.publish(cmd1)
                
                            self.get_logger().info("Detected ID 0: Turning RIGHT")
                
                            self.turn(id)
                   
                            self.detected_count += 1
                            break
                        elif id == 1:
                
                            cmd1.linear.x =0.0
                            self.vel_pub.publish(cmd1)
                
                            self.get_logger().info("Detected ID 1: Turning LEFT")
                            self.turn(id)
                    
                            self.detected_count += 1
                            break
            else :
                self.get_logger().info("tag too far ignoring for now ")


    
    
    def turn (self,id):
        cmd =Twist()
        duration =1.5708/1.5
        if(id == 0):
            cmd.angular.z = -1.5
            start_time =self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 -start_time <duration +0.1:
                self.turning =True
                self.vel_pub.publish(cmd)
                
        else:
            cmd.angular.z = 1.5
            start_time =self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 -start_time <duration +0.1:
                self.vel_pub.publish(cmd)
                self.turning =True
                
        self.turning = False
        cmd.angular.z =0.0 
        self.vel_pub.publish(cmd)
        
        
def main(args =None):
    rclpy.init(args =args)
    node = aruco_nav()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    