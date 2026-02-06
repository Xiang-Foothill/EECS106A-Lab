import rclpy
from rclpy.node import Node
import json
import sys
# Import our custom service
from turtle_patrol_interface.srv import Patrol
from turtlesim.srv import TeleportAbsolute


class TurtlePatrolClient(Node):

    def __init__(self, turtle_name, vel, omega, x, y, theta):
        super().__init__('turtle1_patrol_client')


        self._service_name = '/patrol'

        # Create a client for our Patrol service type
        self._client = self.create_client(Patrol, self._service_name)

        # Wait until the server is up (polling loop; logs once per second)
        self.get_logger().info(f"Waiting for service {self._service_name} ...")
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service {self._service_name} not available, waiting...")

        # get the service
        self.get_logger().info(f"Requesting patrol: turtle_name = {turtle_name} vel={vel}, omega={omega}, x = {x}, y = {y}, theta = {theta}")

        # Build request
        req = Patrol.Request()
        req.turtle_name = turtle_name
        req.vel = vel
        req.omega = omega
        req.x = x
        req.y = y
        req.theta = theta

        # Send request (async under the hood)
        self._future = self._client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    turtle_name = sys.argv[1]
    vel = float(sys.argv[2])
    omega = float(sys.argv[3])
    x = float(sys.argv[4])
    y = float(sys.argv[5])
    theta = float(sys.argv[6])

    node = TurtlePatrolClient(turtle_name = turtle_name, vel = vel, omega = omega, x = x, y = y, theta = theta)

    # Block here until the service responds (simple for teaching)
    rclpy.spin_until_future_complete(node, node._future)

    if node._future.done():
        result = node._future.result()
        if result is not None:
            # Print the Twist returned by the server
            cmd = result.cmd
            node.get_logger().info(
                f"Service response Twist: lin.x={cmd.linear.x:.2f}, ang.z={cmd.angular.z:.2f}"
            )
        else:
            node.get_logger().error("Service call failed: no result returned.")
    else:
        node.get_logger().error("Service call did not complete.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


