#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from coordinate_follower.action import ToDo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class ActionServerNode(Node):
    def __init__(self):
        super().__init__('action_server')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_x = 0.0
        self.current_y = 0.0

        self._action_server = ActionServer(
            self,
            ToDo,
            'todo',
            execute_callback=self.execute_callback
        )

        self.get_logger().info("Action server started")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def compute_angle(self, goal_x, goal_y):
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        return math.atan2(dy, dx)

    def create_twist(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    async def execute_callback(self, goal_handle):
        goal_x = goal_handle.request.x
        goal_y = goal_handle.request.y
        self.get_logger().info(f"Received goal: x={goal_x}, y={goal_y}")

        # 1. Rotate to face goal
        angle = self.compute_angle(goal_x, goal_y)
        self.get_logger().info(f"Turning to angle {math.degrees(angle):.2f}")
        
        twist = self.create_twist(angular_z=0.1245 if angle > 0 else -0.1245)
        start_time = self.get_clock().now().nanoseconds / 1e9
        duration = abs(angle) / 0.1245

        while self.get_clock().now().nanoseconds / 1e9 - start_time < (duration):
            self.vel_pub.publish(twist)
        
        self.vel_pub.publish(self.create_twist(angular_z=0.0))  # stop rotation

        # 2. Move forward towards the goal
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.hypot(dx, dy)
        self.get_logger().info(f"Distance to goal: {distance:.2f}")

        duration1 =distance /0.2
        start_time1 = self.get_clock().now().nanoseconds / 1e9
        while self.get_clock().now().nanoseconds / 1e9 - start_time1 < (duration1+0.1):
            self.vel_pub.publish(self.create_twist(linear_x=0.2))

        self.vel_pub.publish(self.create_twist(linear_x=0.0))

        # 3.rotating back     
        start_time3 = self.get_clock().now().nanoseconds / 1e9
        twist1 = self.create_twist(angular_z=-0.1245 if angle > 0 else 0.1245)
        while self.get_clock().now().nanoseconds / 1e9 - start_time3 < (duration+0.3):
            self.vel_pub.publish(twist1)   
        self.vel_pub.publish(self.create_twist(angular_z=0.0))     

        # 4. Return result
        goal_handle.succeed()
        result = ToDo.Result()
        result.r = self.current_x
        result.s = self.current_y
        self.get_logger().info("Goal reached, result sent")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()





                
                
 