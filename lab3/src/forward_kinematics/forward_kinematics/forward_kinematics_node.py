import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from forward_kinematics.forward_kinematics import ur7e_foward_kinematics_from_angles

class ForwardKinematicsNode(Node):

    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Subscribe to joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Define the expected joint names in the correct kinematic order
        # (Base to End-Effector)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def listener_callback(self, msg):
        """
        Callback function for /joint_states.
        Extracts angles, computes FK, and prints the matrix.
        """
        # 1. Extract joint angles in the correct order
        current_angles = []
        try:
            for target_joint in self.joint_names:
                if target_joint in msg.name:
                    index = msg.name.index(target_joint)
                    current_angles.append(msg.position[index])
                else:
                    self.get_logger().warn(f'Joint {target_joint} not found in JointState message')
                    return

            # Convert to numpy array
            joint_angles = np.array(current_angles)

            # 2. Compute Forward Kinematics
            transformation_matrix = ur7e_foward_kinematics_from_angles(joint_angles)

            # 3. Log/Print the result
            # We use a formatter to make the matrix readable in the console
            np.set_printoptions(precision=3, suppress=True)
            print("\n--- Current End-Effector Pose ----")
            print(transformation_matrix)
            print("-----------------------------------------------")

        except ValueError as e:
            self.get_logger().error(f'Error processing joint states: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()