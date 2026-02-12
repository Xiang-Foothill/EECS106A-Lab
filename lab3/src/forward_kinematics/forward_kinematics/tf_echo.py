import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import sys

class TFEchoNode(Node):
    def __init__(self):
        super().__init__('tf_echo_node')


        if len(sys.argv) < 3:
            self.get_logger().error('Usage: tf_echo <target_frame> <source_frame>')
            sys.exit(1)

        self.target_frame = sys.argv[1]
        self.source_frame = sys.argv[2]


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

            self.print_transform(t)

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Transform not ready: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'Connectivity error: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'Extrapolation error: {e}')

    def print_transform(self, t: TransformStamped):
        """
        Formats and prints the translation and rotation similar to tf2_echo.
        """
        # Translation
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z

        # Rotation (Quaternion)
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        print("-" * 30)
        print(f"At time {t.header.stamp.sec}.{t.header.stamp.nanosec}")
        print(f"- Translation: [{tx:.3f}, {ty:.3f}, {tz:.3f}]")
        print(f"- Rotation: in Quaternion [{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]")

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    
    node = TFEchoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()