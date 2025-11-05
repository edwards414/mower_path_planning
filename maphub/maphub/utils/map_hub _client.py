import rclpy
from rclpy.node import Node 
from boustrophedon_coverage_interfaces.srv import GetZoneList  
from rclpy.callback_groups import ReentrantCallbackGroup

class MapHubClient(Node):
    def __init__(self):
        super().__init__('map_hub_client')
        self.service_callback_group = ReentrantCallbackGroup()
        self.get_risk_zone_list_client = self.create_client(
            GetZoneList, '/get_risk_zone_list',
            callback_group=self.service_callback_group
        )
        self.get_record_zone_list_client = self.create_client(
            GetZoneList, '/get_record_zone_list',
            callback_group=self.service_callback_group
        )
    
