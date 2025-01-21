#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2Servo,MoveIt2
from moveit_msgs.srv import GetCartesianPath
from rclpy.executors import MultiThreadedExecutor,ThreadPoolExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
import time
class Pickandplace(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.get_logger().info("Node Created")

        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        self.callback_group_3=ReentrantCallbackGroup()

        self.moveit2 = MoveIt2(
        node=self,
        joint_names=['Revolute 28', 'Revolute 29', 'Revolute 30', 'Revolute 31', 'Revolute 32','Revolute 33'],
        base_link_name='base_link',
        end_effector_name='end_eff_v1_1',
        group_name='arm',
        callback_group=self.callback_group_3,
        )

        self.cartesian_client=self.create_client(GetCartesianPath,"/compute_cartesian_path")
        self.enable_planner()

        self.timer_=self.create_timer(0.01,self.trajectory_planner,callback_group=self.callback_group_2)

        
    
    def enable_planner(self):
         while not self.cartesian_client.wait_for_service(timeout_sec=2): 
              self.get_logger().info("Waiting for Planner service")
         print("Planner activated")

    def joint_pose_goal(self,pos):
        if pos==1:
            print("moving to goal pose 1 ")
            self.pos_1=[0.1,-0.083,0.529]
            self.ori_1=[0.0,0.0,1.0,0.0]
            self.moveit2.move_to_pose(position=self.pos_1, quat_xyzw=self.ori_1, cartesian=True)
            self.moveit2.wait_until_executed()
            print("moved to goal pose 1 ")
          
        else :
            print("moving to goal pose 2 ")
            self.pos_2=[0.3,-0.083,0.329]
            self.ori_2=[0.0,0.0,1.0,0.0]
            self.moveit2.move_to_pose(position=self.pos_2, quat_xyzw=self.ori_2, cartesian=True)
            self.moveit2.wait_until_executed()
            print("moved to goal pose 2 ")
             

    
    def trajectory_planner(self):
            self.obj = ThreadPoolExecutor(max_workers=1)
            self.obj.submit(self.joint_pose_goal(1))
            time.sleep(2)
            self.obj.submit(self.joint_pose_goal(2))


def main():
    rclpy.init()

    node = Pickandplace()
    # Spin the node in background thread(s)
    executor = MultiThreadedExecutor(5)
    executor.add_node(node)

    executor.spin()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
