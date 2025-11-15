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
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces.action import (
    EspeakAction,
    LlamaAction,
    WhisperAction
)
from .backends import (
    EspeakBackend,
    LlamaBackend,
    WhisperBackend
)

class ML5Server(Node):
    """
    Message Types: https://docs.ros2.org/foxy/api/std_msgs/index-msg.html
    """
    def __init__(self):
        super().__init__('m5_server')
        # Start all of the action servers
        self._espeak_server = ActionServer(
            self,
            EspeakAction,
            'espeak_action',
            self._callback_espeak
        )
        self._llama_server = ActionServer(
            self,
            LlamaAction,
            'llama_action',
            self._callback_llama
        )
        self._whisper_server = ActionServer(
            self,
            WhisperAction,
            'whisper_action',
            self._callback_whisper
        )
        # Initialize model backends
        self._espeak = EspeakBackend()
        self._llama = LlamaBackend()
        self._whisper = WhisperBackend()

    def _call_backend(
            self,
            ctx,
            backend,
            request_attr,
            result_attr,
            feedback_attr,
            ResultType,
            FeedbackType
        ):
        error = ""
        try:
            backend_result = backend(getattr(ctx.request, request_attr))
            ctx.succeed()
        except Exception as e:
            error = str(e)
            ctx.failed()

        feedback = FeedbackType()
        setattr(feedback, feedback_attr, error)
        feedback.feedback = error
        ctx.publish_feedback(feedback)

        result = ResultType()
        setattr(result, result_attr, backend_result)

        return result

    def _callback_espeak(self, ctx):
        self._call_backend(
            ctx,
            self._espeak,
            "text",
            "result",
            "feedback",
            EspeakAction.Result,
            EspeakAction.Feedback
        )

    def _callback_llama(self, ctx):
        self._call_backend(
            ctx,
            self._llama,
            "prompt",
            "response",
            "feedback",
            LlamaAction.Result,
            LlamaAction.Feedback
        )

    def _callback_whisper(self, ctx):
        self._call_backend(
            ctx,
            self._whisper,
            "file_name",
            "text",
            "feedback",
            WhisperAction.Result,
            WhisperAction.Feedback
        )


def main(args=None):
    rclpy.init(args=args)
    server = ML5Server()

    rclpy.spin(server)

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()