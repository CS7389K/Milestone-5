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
import sys
import select

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.action import (
    EspeakAction,
    LlamaAction,
    WhisperAction
)

class ML5Client(Node):
    """
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m5_client')
        self._espeak_client = ActionClient(
            self,
            EspeakAction,
            'espeak_action'
        )
        self._llama_client = ActionClient(
            self,
            LlamaAction,
            'llama_action'
        )
        self._whisper_client = ActionClient(
            self,
            WhisperAction,
            'whisper_action'
        )

    def _request(
            self,
            client,
            ActionType,
            request_attr,
            request_msg
        ):
        request = ActionType.Goal()
        setattr(request, request_attr, request_msg)

        client.wait_for_server()
        send_goal_future = client.send_goal_async(
            request,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._response_callback)

    def _response_callback(self, future):
        goal = future.result()

        if not goal.accepted:
            self.get_logger().info('rejected')
            return

        self.get_logger().info('accepted')
        self._get_result_future = goal.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received error feedback: {0}'.format(feedback.partial_sequence))

    def __call__(
            self,
            client : str,
            request : str
    ):
        assert client in ['espeak', 'llama', 'whisper'], \
            f"Client must be one of 'espeak', 'llama', or 'whisper', got '{client}'"

        if client == 'espeak':
            self._request(
                self._espeak_client,
                EspeakAction,
                'text',
                request
            )
        elif client == 'llama':
            self._request(
                self._llama_client,
                LlamaAction,
                'prompt',
                request
            )
        elif client == 'whisper':
            self._request(
                self._whisper_client,
                WhisperAction,
                'file_name',
                request
            )


def get_user_input(client):
    sys.stdout.write("\nYou (type or 's'/wait→voice): ")
    sys.stdout.flush()
    ready, _, _ = select.select([sys.stdin], [], [], 5)
    if ready:
        line = sys.stdin.readline().strip()
        if   line.lower() in ("exit", "quit"):
            return None
        elif line.lower() == "s":
            client("espeak", "speak in voice now")
        else:
            return line
    # timeout or explicit 's'
    
    return txt


def main(args=None):
    rclpy.init(args=args)
    client = ML5Client()

    while True:
        user_text = get_user_input()
        if user_text is None:
            break

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()