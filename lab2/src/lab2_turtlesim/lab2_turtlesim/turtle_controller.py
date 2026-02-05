#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):

    def __init__(self, turtle_name):
        super().__init__("turtle_controller")
        self.turtle_name = turtle_name
        self.pub = self.create_publisher(Twist, f"/{self.turtle_name}/cmd_vel", 10)
        self.lin = 2.0
        self.ang = 1.0

        self.twist_publisher =  self.create_publisher(Twist, "/my_twist", 10)

    
    def send(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.pub.publish(msg)
        self.twist_publisher.publish(msg)

def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run lab2_turtlesim turtle_controller <turtle_name>")
        return

    turtle_name = sys.argv[1]
    print(turtle_name)

    rclpy.init()
    node = TurtleController(turtle_name)

    try:
        while rclpy.ok():
            key = input("> ").strip().lower()
            if not key:
                continue
            key = key[0]

            if key == "w":
                node.send(node.lin, 0.0)
            elif key == "s":
                node.send(-node.lin, 0.0)
            elif key == "a":
                node.send(0.0, node.ang)
            elif key == "d":
                node.send(0.0, -node.ang)
            elif key == "x":
                node.send(0.0, 0.0)
            elif key == "q":
                node.send(0.0, 0.0)
                break

            # keep ROS responsive (not strictly needed here, but good habit)
            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()