# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# These lines allow us to import rclpy so we can use Python and its Node class
import rclpy
from rclpy.node import Node

# This line imports the built-in string message type that our node will use to structure its data to pass on our topic
from std_msgs.msg import String
from my_chatter_msgs.msg import TimestampString

# We're creating a class called Talker, which is a subclass of Node
class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')

        self.publisher_ = self.create_publisher(
            TimestampString,
            '/user_messages',
            10
        )

        self.get_logger().info('User input publisher started.')

    def run(self):
        while rclpy.ok():
            try:
                text = input("Please enter a line of text and press <Enter>: ")
            except EOFError:
                break

            msg = TimestampString()
            msg.input = text
            now = self.get_clock().now()
            t = now.nanoseconds * 1e-9
            msg.timestamp = t

            self.publisher_.publish(msg)

            self.get_logger().info(
                f'Published: "{text}" at {msg.timestamp}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = UserInputPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
