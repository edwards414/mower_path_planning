import rclpy
from rclpy.node import Node
from boustrophedon_coverage_interfaces.srv import ZoneMapList

class ZoneMapClient(Node):
    def __init__(self):
        super().__init__('zone_map_client')
        self.client = self.create_client(ZoneMapList, '/get_zone_map_list_srv')
        
    def get_zone_maps(self):
        """调用服务获取 zone map 列表"""
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务不可用')
            return None
            
        request = ZoneMapList.Request()  # 空请求
        future = self.client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'收到 {len(response.zone_map_list)} 个 ZoneMap')
            return response.zone_map_list
        else:
            self.get_logger().error('服务调用失败')
            return None

def main():
    rclpy.init()
    client = ZoneMapClient()
    zone_maps = client.get_zone_maps()
    
    if zone_maps:
        for i, zone_map in enumerate(zone_maps):
            print(f"Zone {i}: ID={zone_map.zone_id}")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()