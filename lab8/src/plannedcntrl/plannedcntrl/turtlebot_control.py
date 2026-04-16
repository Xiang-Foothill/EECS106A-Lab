#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import transforms3d.euler as euler
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from plannedcntrl.trajectory import plan_curved_trajectory  # Your existing Bezier planner
import time

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher and TF setup
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Controller gains
        self.Kp = np.diag([0.8, 1.5])
        self.Ki = np.diag([0.0, 0.0])
        self.Kd = np.diag([0.0, 0.1])

        # Subscriber
        self.create_subscription(PointStamped, '/goal_point', self.planning_callback, 10)
        self.timer = self.create_timer(0.5, self.control_loop)

        self.trajectory = None
        self.traj_index = 0
        self.x_i_err = 0.0
        self.yaw_i_err = 0.0
        self.prev_x_err = None
        self.prev_yaw_err = None

        self.get_logger().info('TurtleBot controller node initialized.')

    # ------------------------------------------------------------------
    # Main waypoint controller
    # ------------------------------------------------------------------

    def control_loop(self):
        if not self.trajectory:
            return

        if self.traj_index >= len(self.trajectory):
            self.trajectory = None
            self.traj_index = 0
            self.x_i_err = 0.0
            self.yaw_i_err = 0.0
            self.prev_x_err = None
            self.prev_yaw_err = None
            self.pub.publish(Twist())
            return

        waypoint = self.trajectory[self.traj_index]
        waypoint_pose = PoseStamped()
        # TODO: Fill in waypoint pose message using docs for PoseStamped 
        # (recall what frame this trajectory point is in from your trajectory.py code)
        waypoint_pose.pose.position.x = waypoint[0]
        waypoint_pose.pose.position.y = waypoint[1]
        waypoint_pose.pose.position.z = 0.

        quat = self._quat_from_yaw(waypoint[2])
        waypoint_pose.pose.orientation.x = quat[0]
        waypoint_pose.pose.orientation.y = quat[1]
        waypoint_pose.pose.orientation.z = quat[2]
        waypoint_pose.pose.orientation.w = quat[3]

        # TODO: Find tf and transform waypoint to base_link
        node = rclpy.create_node('turtlebot_controller')
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, node)

        # Keep trying until transform available
        while rclpy.ok():
            try:
                odom_to_base = tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time()) ## TODO: Apply a lookup transform from our world frame to the turtlebot frame
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                node.get_logger().warn('TF lookup failed, retrying...')
                rclpy.spin_once(node, timeout_sec=0.1)
        
        waypoint_base = do_transform_pose(waypoint_pose.pose, odom_to_base)

        # TODO: Calculate proportional error terms including x_err and y_err
        x_err = waypoint_base.position.x
        y_err = waypoint_base.position.y

        q = waypoint_base.orientation
        
        quat_vec = [q.w, q.x, q.y, q.z]
        roll, pitch, yaw_err = euler.quat2euler(quat_vec, axes='sxyz')

        if abs(x_err) < 0.03 and abs(y_err) < 0.03:
            self.traj_index += 1
            print("Waypoint Reached, Now going to waypoint ", self.traj_index)
            self.prev_x_err = None
            self.prev_yaw_err = None
            return

        # TODO: Update derivative and integral error terms (refer to class variables defined in init)
        if self.prev_x_err is not None:
            dx = x_err - self.prev_x_err
        else:
            dx = 0.
        
        if self.prev_yaw_err is not None:
            d_yaw = yaw_err - self.prev_yaw_err
        else:
            d_yaw = 0.

        self.prev_x_err = x_err
        self.prev_yaw_err = yaw_err

        # TODO: Generate control command from error terms
        control_cmd = Twist()
        x_cmd = self.Kp[0][0] * x_err + self.Kd[0][0] * dx
        yaw_cmd = self.Kp[1][1] * yaw_err + self.Kd[1][1] * d_yaw
        control_cmd.linear.x = float(x_cmd)
        control_cmd.angular.z = float(yaw_cmd)
        self.pub.publish(control_cmd)
  

    # ------------------------------------------------------------------
    # Callback when goal point is published
    # ------------------------------------------------------------------
    def planning_callback(self, msg: PointStamped):
        if self.trajectory:
            return
        self.trajectory = plan_curved_trajectory([msg.point.x, msg.point.y])
        self.traj_index = 0
        self.x_i_err = 0.0
        self.yaw_i_err = 0.0
        self.prev_x_err = None
        self.prev_yaw_err = None

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_from_yaw(yaw):
        """Return quaternion (x, y, z, w) from yaw angle."""
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
