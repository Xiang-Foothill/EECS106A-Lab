#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class ConstantTransformPublisher(Node):
    def __init__(self):
        super().__init__('constant_tf_publisher')
        self.br = StaticTransformBroadcaster(self)

        # Homogeneous transform G_wrist_3_link->camera_depth_optical
        G = np.array([[1, 0, 0, -0.025],
                      [0, 1, 0, 0.13],
                      [0, 0, 1, 0.0],
                      [0, 0, 0, 1.0]
        ])

        # Create TransformStamped
        self.transform = TransformStamped()

        # ------------------------------------------
        # TODO: Fill out TransformStamped message
        # ------------------------------------------
        self.transform.child_frame_id = 'camera_depth_optical_frame'
        self.transform.header.frame_id = 'wrist_3_link'

        r = R.from_matrix([
                [1., 0., 0],
                [0., 1., 0],
                [0, 0, 1.]])
        quaternion = r.as_quat()
        

        self.transform.transform.translation.x = -0.025
        self.transform.transform.translation.y = 0.13
        self.transform.transform.translation.z = 0.0

        self.transform.transform.rotation.x = quaternion[0]
        self.transform.transform.rotation.y = quaternion[1]
        self.transform.transform.rotation.z = quaternion[2]
        self.transform.transform.rotation.w = quaternion[3]
        # Convert rotation matrix to quaternion (x, y, z, w)

        # Populate TransformStamped

        self.get_logger().info(f"Broadcasting transform:\n{G}\nQuaternion: {quaternion}")


        self.timer = self.create_timer(0.05, self.broadcast_tf)

    def broadcast_tf(self):
        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform)

def main():
    rclpy.init()
    node = ConstantTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
