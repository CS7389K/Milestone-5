# Code taken from:
# https://github.com/ros2/demos/blob/1d01c8e3d06644c0d706ef83697a68efda7d0ad4/action_tutorials/action_tutorials_py/action_tutorials_py/fibonacci_action_server.py
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
# limitations under the License.import time

import time

from interfaces.action import Action

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class ML5Server(Node):

    def __init__(self):
        super().__init__('ml5_server')
        self._action_server = ActionServer(
            self,
            Action,
            'action',
            self.execute_callback
        )

    def process(self, goal):
        pass


def main(args=None):
    rclpy.init(args=args)
    server = ML5Server()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()