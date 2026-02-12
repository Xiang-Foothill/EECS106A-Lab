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

# Key mappings
INCREMENT_KEYS = ['1','2','3','4','5','6']
DECREMENT_KEYS = ['q','w','e','r','t','y']
JOINT_STEP = 0.15 # radians per key press

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

        self.joint_positions = [0.0] * 6
        self.got_joint_states = False  # Failsafe: don't publish until joint states received
        
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_validated', 10)
        
        self.running = True
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    def pub_trajectory()
        
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = self.new_positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 1
        traj.points.append(point)
        self.pub.publish(traj)

        self.joint_positions = new_positions

def main(args=None):
    rclpy.init(args=args)
    new_pos = [args[1], args[2], args[3], args[4], args[5], args[6]]
    node = TrajController(new_pos = new_pos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.running = False
        print("\nExiting keyboard controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
