#!/usr/bin/env python3

"""ROS 2 joystick teleoperation node for the mower blade controller."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

from mower_teleop.blade_teleop_logic import BladeTeleopConfig
from mower_teleop.blade_teleop_logic import BladeTeleopController


class BladeTeleopJoy(Node):
    """Translate joystick input into blade controller commands."""

    def __init__(self) -> None:
        super().__init__('blade_teleop_joy')

        publish_rate = max(float(self.declare_parameter('publish_rate', 20.0).value), 1.0)
        config = BladeTeleopConfig(
            blade_axis_index=int(self.declare_parameter('blade_axis_index', 5).value),
            axis_released_value=float(
                self.declare_parameter('axis_released_value', 1.0).value
            ),
            axis_pressed_value=float(
                self.declare_parameter('axis_pressed_value', -1.0).value
            ),
            enable_button_index=int(
                self.declare_parameter('enable_button_index', 4).value
            ),
            estop_button_index=int(self.declare_parameter('estop_button_index', 0).value),
            max_blade_command=max(
                float(self.declare_parameter('max_blade_command', 100.0).value),
                0.0,
            ),
            joy_timeout=max(float(self.declare_parameter('joy_timeout', 0.3).value), 0.0),
            command_ramp_per_sec=max(
                float(self.declare_parameter('command_ramp_per_sec', 200.0).value),
                0.0,
            ),
            axis_release_tolerance=max(
                float(
                    self.declare_parameter(
                        'axis_release_tolerance', 0.1
                    ).value
                ),
                0.0,
            ),
        )

        self.publish_rate = publish_rate
        self.controller = BladeTeleopController(config)
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/mower_blade_controller/commands',
            10,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self._handle_joy,
            10,
        )
        self._last_tick = time.monotonic()
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self._handle_timer,
        )

        self.get_logger().info(
            'Blade teleop ready: axis=%d enable_button=%d estop_button=%d '
            'max_command=%.1f timeout=%.2fs rate=%.1fHz'
            % (
                config.blade_axis_index,
                config.enable_button_index,
                config.estop_button_index,
                config.max_blade_command,
                config.joy_timeout,
                self.publish_rate,
            )
        )

    def _publish_command(self, value: float) -> None:
        message = Float64MultiArray()
        message.data = [float(value)]
        self.command_publisher.publish(message)

    def _log_event(self, event: str) -> None:
        if event == 'estop':
            self.get_logger().warn('Blade emergency stop triggered')
        elif event == 'timeout':
            self.get_logger().warn('Joystick timeout, blade command reset to zero')
        elif event == 'deadman_released':
            self.get_logger().info('Blade deadman released; command is zero')
        elif event == 'not_armed':
            self.get_logger().warn(
                'Blade is not armed: release the trigger and deadman first'
            )
        elif event == 'invalid_axis':
            self.get_logger().error(
                'Blade axis is missing or non-finite; command is zero'
            )

    def _handle_joy(self, message: Joy) -> None:
        event = self.controller.handle_joy(
            axes=message.axes,
            buttons=message.buttons,
            now=time.monotonic(),
        )
        if event is not None:
            self._log_event(event)
            if event == 'estop':
                self._publish_command(0.0)

    def _handle_timer(self) -> None:
        now = time.monotonic()
        dt = max(now - self._last_tick, 0.0)
        self._last_tick = now

        command, event = self.controller.update(now, dt)
        if event is not None:
            self._log_event(event)

        self._publish_command(command)


def main() -> None:
    """Run the blade joystick teleoperation node."""
    rclpy.init()

    node = BladeTeleopJoy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
