"""Zone map client for retrieving zone maps from service."""

# Copyright 2024 fxrbindi
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
from boustrophedon_coverage_interfaces.msg import ZoneMap
from boustrophedon_coverage_interfaces.srv import ZoneMapList

import rclpy
from rclpy.node import Node


class ZoneMapClient(Node):
    """Client for retrieving zone maps from the zone map service."""

    def __init__(self):
        """Initialize the zone map client."""
        super().__init__('zone_map_client')
        self.client = self.create_client(
            ZoneMapList, '/get_zone_map_list_srv'
        )

    def get_zone_maps(self) -> list[ZoneMap]:
        """
        Get the zone map list.

        Returns
        -------
        list[ZoneMap]
            The zone map list.

        """
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务不可用')
            return None

        request = ZoneMapList.Request()  # 空请求
        future = self.client.call_async(request)

        # 使用獨立的 executor 等待 future，避免與主節點的 spin 衝突
        # （直接呼叫 rclpy.spin_until_future_complete 會導致 "Executor is already spinning"）
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=10.0)
        finally:
            executor.remove_node(self)
            executor.shutdown()

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'收到 {len(response.zone_map_list)} 个 ZoneMap'
            )
            return response.zone_map_list
        else:
            self.get_logger().error('服务调用失败')
            return None


def main():
    """Run the zone map client node."""
    rclpy.init()
    client = ZoneMapClient()
    zone_maps = client.get_zone_maps()

    if zone_maps:
        for i, zone_map in enumerate(zone_maps):
            print(f'Zone {i}: ID={zone_map.zone_id}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
