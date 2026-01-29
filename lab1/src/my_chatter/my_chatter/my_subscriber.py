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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from my_chatter_msgs.msg import TimestampString

class UserMessageSubscriber(Node):
    def __init__(self):
        super().__init__('user_message_subscriber')

        self.subscription = self.create_subscription(
            TimestampString,
            '/user_messages',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('User message subscriber started. Listening on /user_messages')

    def listener_callback(self, msg: TimestampString):
        # "Received at" timestamp is generated HERE (subscriber-side), not in the message.
        now = self.get_clock().now()
        received_at = now.nanoseconds * 1e-9  # float seconds

        # Your publisher uses msg.input (string) and msg.timestamp (float seconds)
        message_text = msg.input
        sent_at = msg.timestamp

        print(f"Message: {message_text}, Sent at: {sent_at}, Received at: {received_at}")


def main(args=None):
    rclpy.init(args=args)
    node = UserMessageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
