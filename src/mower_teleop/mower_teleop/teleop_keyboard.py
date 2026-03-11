#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import sys, select, termios, tty

KEY_HINTS = """
╔══════════════════════════════════════╗
║      Mower Teleop - 鍵盤控制說明      ║
╠══════════════════════════════════════╣
║   W   ：前進                         ║
║   S   ：後退                         ║
║   A   ：左轉                         ║
║   D   ：右轉                         ║
║ SPACE ：停止（發送零速）               ║
║   Q   ：離開                         ║
╚══════════════════════════════════════╝
"""

class Teleop(Node):

    def __init__(self):
        super().__init__('mower_teleop')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.linear = 0.3
        self.angular = 1.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(KEY_HINTS)

        while rclpy.ok():

            key = self.get_key()

            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'

            if key == 'w':
                twist.twist.linear.x = self.linear
                print(f'[前進]  → linear: {self.linear:.2f} | angular: {self.angular:.2f}')

            elif key == 's':
                twist.twist.linear.x = -self.linear
                print(f'[後退]  → linear: {-self.linear:.2f} | angular: {self.angular:.2f}')

            elif key == 'a':
                twist.twist.angular.z = self.angular
                print(f'[左轉]  → linear: {self.linear:.2f} | angular: {self.angular:.2f}')

            elif key == 'd':
                twist.twist.angular.z = -self.angular
                print(f'[右轉]  → linear: {self.linear:.2f} | angular: {-self.angular:.2f}')

            elif key == ' ':
                print(f'[停止]  → linear: {self.linear:.2f} | angular: {self.angular:.2f}')

            elif key in ('q', 'Q', '\x03'):   # q / Ctrl+C
                print('\n[離開] 再見！')
                break

            else:
                continue

            self.pub.publish(twist)


def main():
    rclpy.init()

    node = Teleop()
    node.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()