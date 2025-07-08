#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from coordinate_follower.action import ToDo
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
import time
class action_client(Node):
    def __init__(self):
        super().__init__("action_client")
        self.acting_client =ActionClient(self,ToDo,"todo")
        self.get_logger().info("action client has started")
        self.i= 0
        self.j=1

    def send_goal(self):
        #wait for server
        
        self.acting_client.wait_for_server()
        #create goal
        goal = ToDo.Goal()
        with open("/home/atrij/kratos_ws/src/coordinate_follower/Values.txt")as f: 
            v =f.readline()                                            
            value =v.split()
            length =len(value)
            for i in range(0,length,2):
                goal.x =float(value[self.i])
                goal.y =float(value[(self.i)+1])
                self.acting_client.wait_for_server()
                self.get_logger().info(f'sending GOAL {self.j} to server')
                send_goal_future = self.acting_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self, send_goal_future)
                goal_handle = send_goal_future.result()
                if not goal_handle.accepted:
                    self.get_logger().warn(f"Goal {i} was rejected by server")
                    continue
                self.i +=2
                
                self.get_logger().info(f"Goal{self.j} accepted,waiting for result")
                self.j +=1
                get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, get_result_future)
                result = get_result_future.result().result
                self.get_logger().info(f"Result {i}: r={result.r:.2f}, s={result.s:.2f}")
                
                
    def goal_result_callback(self ,future):
        result =future.result().result
        self.get_logger().info("result :"+ str(result.r) +"   "+ str(result.s))
        


def main (args =None):
    rclpy.init(args =args)
    node = action_client()
    node.send_goal()
    while rclpy.ok():
        rclpy.spin_until_future_complete(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

       
