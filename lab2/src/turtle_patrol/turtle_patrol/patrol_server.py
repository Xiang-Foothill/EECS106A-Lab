import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_patrol_interface.srv import Patrol


class Turtle1PatrolServer(Node):
    def __init__(self):
        super().__init__('turtle1_patrol_server')
        self._srv = self.create_service(Patrol, '/patrol', self.patrol_callback)

        # Current commanded speeds (what timer publishes)
        self._lin = 0.0
        self._ang = 0.0

        # Timer: publish current speeds at 10 Hz
        self._pub_timer = self.create_timer(0.1, self._publish_current_cmd)

        self.get_logger().info('Turtle1PatrolServer ready (continuous publish mode).')
        self.turtle_pub_dict = {}
        self.turtle_cmd_dict = {}

    # -------------------------------------------------------
    # Timer publishes current Twist
    # -------------------------------------------------------
    def _publish_current_cmd(self):
        for name in self.turtle_pub_dict:

            msg = Twist()
            msg.linear.x = self.turtle_cmd_dict[name][0]
            msg.angular.z = self.turtle_cmd_dict[name][1]
            self.turtle_pub_dict[name].publish(msg)

    # -------------------------------------------------------
    # Service callback: update speeds
    # -------------------------------------------------------
    def patrol_callback(self, request: Patrol.Request, response: Patrol.Response):
        teleport_service_name = f'/{request.turtle_name}/teleport_absolute'

        # Create a client for our Teleport service type
        self._client = self.create_client(TeleportAbsolute, teleport_service_name)

        # Build request
        tele_req = TeleportAbsolute.Request()
        tele_req.x = request.x
        tele_req.y = request.y
        tele_req.theta = request.theta

        # Send request (async under the hood)
        self._future = self._client.call_async(tele_req)

        if request.turtle_name not in self.turtle_pub_dict:
            self.turtle_pub_dict[request.turtle_name] = self.create_publisher(Twist, f'/{request.turtle_name}/cmd_vel', 10)
        self.turtle_cmd_dict[request.turtle_name] = [request.vel, request.omega]

        self.get_logger().info(
            f"Patrol request: turtle_name = {request.turtle_name} vel={request.vel:.2f}, omega={request.omega:.2f}"
        )

        # Prepare response Twist reflecting current command
        cmd = Twist()
        cmd.linear.x = request.vel
        cmd.angular.z = request.omega
        response.cmd = cmd

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Turtle1PatrolServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
