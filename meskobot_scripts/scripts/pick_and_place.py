#!/usr/bin/env python3

from meskobot_scripts.srv import Sol 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        # Create publisher for the trajectory
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Subscribe to the joint states topic
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Store current joint positions
        self.current_joint_positions = None

    def joint_state_callback(self, msg):
        # Get current joint positions from the joint state message
        self.current_joint_positions = msg.position
        self.get_logger().info(f"Received joint positions: {self.current_joint_positions}")

    def publish_trajectory(self, j1, j2, j3, j4, j5, j6):
        if self.current_joint_positions is None:
            self.get_logger().warn("Waiting for joint state message...")
            return

        # Create the trajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['Revolute 28', 'Revolute 29', 'Revolute 30', 'Revolute 31', 'Revolute 32', 'Revolute 33']
        
        # Define the trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.5,0.5,0.5,0.5,0.5,0.5]
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6

        # Maximum joint velocity (in rad/s)
        max_velocity = 1.5  # m/s

        # Calculate the time for each joint
        times = []
        for i, desired_position in enumerate(point.positions):
            current_position = self.current_joint_positions[i] if i < len(self.current_joint_positions) else 0.0
            delta_position = abs(desired_position - current_position)
            time_required = delta_position / max_velocity
            times.append(time_required)

        # Take the maximum time as the total trajectory time
        max_time = max(times)

        # Set time_from_start
        point.time_from_start.sec = int(max_time)
        point.time_from_start.nanosec = int((max_time - int(max_time)) * 1e9)  # Fractional part for nanoseconds

        # Add the point to the trajectory message
        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info(f"Published trajectory with time from start: {max_time} seconds")
        print(point.positions)

# ROS2 entry point
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()

    # For testing purposes, let's publish a trajectory after receiving joint states
    rclpy.spin_once(node)

    # Example usage of publish_trajectory with target positions
    node.publish_trajectory(1.57,0.785,-2.355,1.57,1.57,0.0)
    time.sleep(10)
    rclpy.spin_once(node)
    
    node.publish_trajectory(2.2681306554183385, -0.7681352559139016, -1.2739398036912335, 2.0419798911121703, 0.8687706615044757, 0.0)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

