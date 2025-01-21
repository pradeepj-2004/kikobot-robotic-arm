#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.subscriber_=self.create_subscription(Float32MultiArray,"/ik_topic",self.callback_func,10)

        
        # Publish a trajectory after 2 seconds
        self.timer = self.create_timer(0.5, self.publish_trajectory)
    
    def callback_func(self,msg):
        self.joint_1=msg.data[0]
        self.joint_2=msg.data[1]
        self.joint_3=msg.data[2]
        self.joint_4=msg.data[3]
        self.joint_5=msg.data[4]
        self.joint_6=msg.data[5]

    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['Revolute 28', 'Revolute 29', 'Revolute 30', 'Revolute 31', 'Revolute 32','Revolute 33']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0,0.785,-2.355,1.57,1.57,0.0] # Desired positions for each joint
        # point.positions=[0.0]*6
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start.sec = 1

        trajectory_msg.points.append(point)
        self.publisher_.publish(trajectory_msg)
        # self.get_logger().info(point.positions)
        print(point.positions)

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
