#!/usr/bin/env python3
"""
Example of computing Inverse Kinematics.
- ros2 run pymoveit2 ex_ik.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
- ros2 run pymoveit2 ex_ik.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p synchronous:=False
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_ik")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.360192,-0.03757391,0.05301151+0.2])
    node.declare_parameter("quat_xyzw", [ 0.0, 0.7068252, 0.0, 0.7073883])
    node.declare_parameter("synchronous", True)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=['Revolute 28','Revolute 29','Revolute 30','Revolute 31','Revolute 32','Revolute 33'],
        base_link_name='base_link',
        end_effector_name='end_eff_v1_1',
        group_name='arm',
        callback_group=callback_group,
    )
    # moveit2 = MoveIt2(
    #     node=node,
    #     joint_names=['joint_1','joint_2','joint_3','joint_4','joint_5'],
    #     base_link_name='base_link',
    #     end_effector_name='gripper_left',
    #     group_name='gripper',
    #     callback_group=callback_group,
    # )


    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    synchronous = node.get_parameter("synchronous").get_parameter_value().bool_value

    # Move to joint configuration
    node.get_logger().info(
        f"Computing IK for {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    retval = None
    if synchronous:
        retval = moveit2.compute_ik(position, quat_xyzw)
    else:
        future = moveit2.compute_ik_async(position, quat_xyzw)
        if future is not None:
            rate = node.create_rate(10)
            while not future.done():
                rate.sleep()
            retval = moveit2.get_compute_ik_result(future)
    if retval is None:
        print("Failed.")
    else:
        print("Succeeded. Result: " + str(retval))

    rclpy.shutdown()
    executor_thread.join()
    exit(0)

if __name__ == "__main__":
    main()
