#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import tty
import termios
import threading
import select
import time

class TrajController(Node):
    def __init__(self, new_pos):
        super().__init__('ur7e_keyboard_controller')
        
        self.joint_names = [
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'shoulder_pan_joint',
        ]
        self.new_positions = new_pos
        print(f"target positions: {self.new_positions}")

        self.joint_positions = [0.0] * 6
        self.got_joint_states = False  # Failsafe: don't publish until joint states received
        
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_validated', 10)
    
    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        self.got_joint_states = True

    def pub_trajectory(self):
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.new_positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    new_pos = [sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6]]
    new_pos = [float(pos) for pos in new_pos]
    node = TrajController(new_pos = new_pos)
    node.pub_trajectory()
    try:
        rclpy.spin(node)
        time.sleep(2)
    except KeyboardInterrupt:
        node.running = False
        print("\nExiting keyboard controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
