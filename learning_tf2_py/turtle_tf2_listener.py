# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time, CONVERSION_CONSTANT

import pandas as pd


class FrameListener(Node):

    def __init__(self, queue_size=100):
        super().__init__('turtle_tf2_frame_listener')

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'dummy0')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            queue_size)
        self.subscription  # prevent unused variable warning

        self.counter = 0

        self.delay_data = []


        self.get_logger().info(f'Running with queue_size={queue_size}')

    def listener_callback(self, msg: TFMessage):
        current_time = self.get_clock().now()  # type: Time
        last_transform = msg.transforms[-1]  # type: TransformStamped
        last_transform_time = Time.from_msg(last_transform.header.stamp)  # type: Time

        delay = current_time - last_transform_time
        delay_ns = delay.nanoseconds
        secs_nsecs = (delay_ns // CONVERSION_CONSTANT, (delay_ns % CONVERSION_CONSTANT) / CONVERSION_CONSTANT)
        secs = sum(secs_nsecs)
        self.get_logger().info(f'{self.counter}: Delay: "{secs}"')

        self.counter += 1
        self.delay_data += [secs]


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Measure difference between received stamp and current time")
    parser.add_argument(
        "--queue-size",
        "--queue_size",
        type=int,
        default=100,
    )
    args = parser.parse_args()
    print(args)

    rclpy.init()
    node = FrameListener(args.queue_size)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        df = pd.DataFrame({'delay': node.delay_data})
        print(df.describe())

    rclpy.shutdown()
