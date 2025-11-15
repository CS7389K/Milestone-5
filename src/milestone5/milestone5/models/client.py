# Code taken from:
# https://github.com/ros2/demos/blob/1d01c8e3d06644c0d706ef83697a68efda7d0ad4/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_client.py
#
#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from interfaces.action import Action

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class ML5Client(Node):

    def __init__(self):
        super().__init__('ml5_client')
        self._action_client = ActionClient(self, Action, 'action')

    def send_goal(self, order):
        msg = Action.Goal()
        # Add msg properties

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal = future.result()

        if not goal.accepted:
            self.get_logger().info('rejected')
            return

        self.get_logger().info('accepted')
        self._get_result_future = goal.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = ML5Client()
    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()