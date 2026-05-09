#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from mower_mission.battery_simulator import BatteryDistanceSimulator


class BatterySimulatorNode(Node):
    """Publish simulated BatteryState drained by odometry distance."""

    def __init__(self):
        super().__init__('battery_simulator')

        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('initial_percentage', 100.0)
        self.declare_parameter('meters_per_percent', 100.0)
        self.declare_parameter('min_delta_m', 0.0)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.simulator = BatteryDistanceSimulator(
            initial_percentage=self.get_parameter(
                'initial_percentage'
            ).value,
            meters_per_percent=self.get_parameter(
                'meters_per_percent'
            ).value,
            min_delta_m=self.get_parameter('min_delta_m').value,
        )
        self.odom_topic = self.get_parameter('odom_topic').value
        self.battery_topic = self.get_parameter('battery_topic').value

        self.battery_pub = self.create_publisher(
            BatteryState,
            self.battery_topic,
            10,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        timer_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 1.0
        self.create_timer(timer_period, self.publish_battery_state)
        self.publish_battery_state()

        self.get_logger().info(
            'Battery simulator started: '
            f'100m -> 1%, odom={self.odom_topic}, '
            f'topic={self.battery_topic}'
        )

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        previous_percentage = self.simulator.percentage
        percentage = self.simulator.update_position(position.x, position.y)
        if int(previous_percentage) != int(percentage):
            self.get_logger().info(
                f'battery={percentage:.1f}% '
                f'distance={self.simulator.distance_m:.2f}m'
            )
        self.publish_battery_state()

    def publish_battery_state(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.percentage = self.simulator.percentage_fraction
        msg.present = True
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )
        msg.power_supply_status = self._power_supply_status()
        self.battery_pub.publish(msg)

    def _power_supply_status(self) -> int:
        if self.simulator.percentage <= 0.0:
            return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        if self.simulator.distance_m <= 0.0:
            return BatteryState.POWER_SUPPLY_STATUS_FULL
        return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
