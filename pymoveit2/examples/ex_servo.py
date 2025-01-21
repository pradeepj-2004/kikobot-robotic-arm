#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
- ros2 run pymoveit2 ex_servo.py
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger

from pymoveit2 import MoveIt2Servo,MoveIt2
from pymoveit2.robots import panda
from geometry_msgs.msg import TwistStamped,TransformStamped


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    node.twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    node.start_service = node.create_client(srv_type=Trigger,srv_name="/servo_node/start_servo")

    def enable():
        
        return node.start_service.call_async(Trigger.Request())

    def servo_circular_motion():

            node.twist_msg = TwistStamped()
            node.twist_msg.header.frame_id="base_link"
            node.twist_msg.header.stamp = node.get_clock().now().to_msg()
            node.twist_msg.twist.linear.x = 0.1
            node.twist_msg.twist.linear.y = -0.1
            node.twist_msg.twist.linear.z = 0.0
            node.twist_msg.twist.angular.x = 0.0
            node.twist_msg.twist.angular.y = 0.0
            node.twist_msg.twist.angular.z = 0.0
            node.twist_pub.publish(node.twist_msg)
            print('hi')


    # Create timer for moving in a circular motion
    enable()
    node.create_timer(0.02, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
