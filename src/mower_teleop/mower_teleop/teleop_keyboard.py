#!/usr/bin/env python3

import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

KEY_HINTS = """
╔══════════════════════════════════════╗
║      Mower Teleop - 鍵盤控制說明      ║
╠══════════════════════════════════════╣
║   W   ：提高前進速度                  ║
║   S   ：降低速度 / 後退               ║
║   A   ：向左修正（放開回正）           ║
║   D   ：向右修正（放開回正）           ║
║ SPACE ：急停（立即歸零）               ║
║   Q   ：離開                         ║
╚══════════════════════════════════════╝
"""


class Teleop(Node):

    def __init__(self):
        super().__init__('mower_teleop')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        self.max_linear_speed = float(
            self.declare_parameter('max_linear_speed', 0.8).value
        )
        self.max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 1.5).value
        )
        self.linear_step = float(self.declare_parameter('linear_step', 0.15).value)
        self.angular_step = float(self.declare_parameter('angular_step', 0.45).value)
        self.linear_acceleration = float(
            self.declare_parameter('linear_acceleration', 0.8).value
        )
        self.angular_acceleration = float(
            self.declare_parameter('angular_acceleration', 3.0).value
        )
        self.publish_rate = max(
            float(self.declare_parameter('publish_rate', 20.0).value),
            1.0,
        )
        self.steering_timeout = max(
            float(self.declare_parameter('steering_timeout', 0.25).value),
            0.0,
        )

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_steering_input = 0.0

    def get_key(self, timeout):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    @staticmethod
    def approach(current, target, max_delta):
        if current < target:
            return min(current + max_delta, target)
        return max(current - max_delta, target)

    def publish_twist(self, linear_x, angular_z):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        twist.twist.linear.x = linear_x
        twist.twist.angular.z = angular_z
        self.pub.publish(twist)

    def print_status(self, label):
        print(
            f'[{label}] '
            f'target linear: {self.target_linear:.2f} m/s | '
            f'target angular: {self.target_angular:.2f} rad/s | '
            f'current linear: {self.current_linear:.2f} m/s | '
            f'current angular: {self.current_angular:.2f} rad/s'
        )

    def emergency_stop(self, label='停止'):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.publish_twist(0.0, 0.0)
        self.print_status(label)

    def handle_key(self, key, now):
        if key in ('w', 'W'):
            self.target_linear = self.clamp(
                self.target_linear + self.linear_step,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            self.target_angular = 0.0
            self.print_status('加速前進')
            return False

        if key in ('s', 'S'):
            self.target_linear = self.clamp(
                self.target_linear - self.linear_step,
                -self.max_linear_speed,
                self.max_linear_speed,
            )
            self.target_angular = 0.0
            self.print_status('減速 / 後退')
            return False

        if key in ('a', 'A'):
            self.target_angular = self.clamp(
                self.target_angular + self.angular_step,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.last_steering_input = now
            self.print_status('左修正')
            return False

        if key in ('d', 'D'):
            self.target_angular = self.clamp(
                self.target_angular - self.angular_step,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self.last_steering_input = now
            self.print_status('右修正')
            return False

        if key == ' ':
            self.emergency_stop()
            return False

        if key in ('q', 'Q', '\x03'):
            print('\n[離開] 再見！')
            return True

        return False

    def run(self):
        print(KEY_HINTS)
        self.print_status('初始狀態')

        period = 1.0 / self.publish_rate
        last_tick = time.monotonic()

        try:
            while rclpy.ok():
                key = self.get_key(period)
                now = time.monotonic()
                dt = max(now - last_tick, 1e-3)
                last_tick = now

                if self.handle_key(key, now):
                    break

                if now - self.last_steering_input > self.steering_timeout:
                    self.target_angular = 0.0

                self.current_linear = self.approach(
                    self.current_linear,
                    self.target_linear,
                    self.linear_acceleration * dt,
                )
                self.current_angular = self.approach(
                    self.current_angular,
                    self.target_angular,
                    self.angular_acceleration * dt,
                )

                self.publish_twist(self.current_linear, self.current_angular)
        finally:
            self.emergency_stop('結束歸零')
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()

    node = Teleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
