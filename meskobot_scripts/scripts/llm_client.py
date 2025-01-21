#!/usr/bin/env python3

from threading import Thread
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from pymoveit2 import MoveIt2Servo,MoveIt2
from geometry_msgs.msg import TwistStamped,TransformStamped
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.executors import MultiThreadedExecutor,ThreadPoolExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
import pickle,time
from custom_service.srv import Actions 

class Pickandplace(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.get_logger().info("Node Created")

        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        self.callback_group_3=ReentrantCallbackGroup()

        self.pos_1=[0.360192,-0.03757391,0.05301151+0.2]
        # self.quat=[]
        self.client_side =self.create_client(Actions,'/llm_server',callback_group=self.callback_group_1)
        self.timer_=self.create_timer(0.01,self.trajectory_planner,callback_group=self.callback_group_2)

        self.moveit2 = MoveIt2(
        node=self,
        joint_names=['Revolute 28', 'Revolute 29', 'Revolute 30', 'Revolute 31', 'Revolute 32','Revolute 33'],
        base_link_name='base_link',
        end_effector_name='end_eff_v1_1',
        group_name='arm',
        callback_group=self.callback_group_3,
        )
    
        self.declare_parameter("quat_xyzw", [ 0.0, 0.7068252, 0.0, 0.7073883])

     
    def joint_pose_goal(self):
        quat_xyzw = self.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        # print(self.pos_1)
        self.moveit2.move_to_pose(position=self.pos_1, quat_xyzw=quat_xyzw, cartesian=True)
        self.moveit2.wait_until_executed()

    
    def trajectory_planner(self):

        request = Actions.Request()
        while not self.client_side.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('LLM Server not running, waiting again...')
        request.prompt="Pick the red"
        future = self.client_side.call_async(request)
        while not future.done():
            a=1
    
        result = future.result()

        if result:
            # self.get_logger().info(f"Response received: {result.actions}")
            # print(result.actions[0],result.actions[1],result.actions[2])
            self.pos_1[0]= self.pos_1[0]+result.actions[0]
            self.pos_1[1]= self.pos_1[1]-result.actions[1]
            self.pos_1[2]= self.pos_1[2]-result.actions[2]
            # print(result.actions[6])
            self.obj = ThreadPoolExecutor(max_workers=1)
            self.obj.submit(self.joint_pose_goal())

def main():
    rclpy.init()

    # Create node for this example
    node = Pickandplace()

    # Spin the node in background thread(s)
    executor = MultiThreadedExecutor(5)
    executor.add_node(node)

    executor.spin()
    executor.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
