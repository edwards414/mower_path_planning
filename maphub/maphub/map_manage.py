import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import cv2
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from boustrophedon_coverage_interfaces.srv import GetZoneList


class MapManage(Node):
    def __init__(self):
        super().__init__('map_manage')
        self.get_logger().info("map_manage init")

        self.set_parameters([
            Parameter('use_sim_time',Parameter.Type.BOOL,True)
        ])

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_service(Trigger, '/create_risk_map', self.create_risk_map_srv)
        self.create_service(Trigger, '/create_free_space', self.create_free_space_srv)

        self.service_callback_group = ReentrantCallbackGroup()
        self.get_risk_zone_list_client = self.create_client(
            GetZoneList, '/get_risk_zone_list',
            callback_group=self.service_callback_group
        )
        self.get_record_zone_list_client = self.create_client(
            GetZoneList, '/get_record_zone_list',
            callback_group=self.service_callback_group
        )

         #發布區
        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', 1)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', 1)
        self.risk_map_pub = self.create_publisher(OccupancyGrid, '/risk_map', 1)
 
        self.latest_map = None #

        self.zone_list = []
        self.base_map = None

    def create_risk_map_srv(self, req, res):
        """創建風險地圖服務"""
        self.get_logger().info("(service)create_risk_map_srv call")
        # 立即返回接受狀態，然後異步處理
        res.success = True
        res.message = "開始創建風險地圖，請稍候..."
        
        # 啟動異步處理
        self._start_create_risk_map_async()
        
        return res
    
    def _start_create_risk_map_async(self):
        """異步創建風險地圖"""
        try:
            # 等待風險區域列表服務可用
            if not self.get_risk_zone_list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("風險區域列表服務不可用")
                return
            
            # 調用獲取風險區域列表服務
            risk_zone_req = GetZoneList.Request()
            future = self.get_risk_zone_list_client.call_async(risk_zone_req)
            
            # 添加完成回調
            future.add_done_callback(self._handle_risk_zone_list_response)
            
        except Exception as e:
            self.get_logger().error(f"啟動異步創建風險地圖時發生錯誤: {e}")
    
    def _handle_risk_zone_list_response(self, future):
        """處理風險區域列表服務響應"""
        try:
            if not future.done():
                self.get_logger().error("獲取風險區域列表超時")
                return
            
            risk_zone_response = future.result()
            
            if not risk_zone_response.success:
                self.get_logger().error(f"獲取風險區域列表失敗: {risk_zone_response.message}")
                return
            
            # 生成風險地圖
            risk_map = self._generate_risk_map(risk_zone_response.zone_list)
            
            if risk_map is not None:
                self.risk_map_pub.publish(risk_map)
                self.get_logger().info(f"成功創建風險地圖，包含 {len(risk_zone_response.zone_list.markers)} 個風險區域")
            else:
                self.get_logger().error("生成風險地圖失敗")
            
        except Exception as e:
            self.get_logger().error(f"處理風險區域列表響應時發生錯誤: {e}")

    def _generate_risk_map(self, risk_zones):
        """根據基礎地圖和風險區域生成風險地圖，使用與free_space相同的原點和分辨率"""
        try:
            # 創建風險地圖，使用與base_map相同的參數（這樣就與free_space保持一致）
            risk_map = OccupancyGrid()
            risk_map.header = self.base_map.header
            risk_map.header.frame_id = 'map'
            risk_map.info = self.base_map.info  # 直接使用base_map的info，確保原點和分辨率一致
            
            W = self.base_map.info.width
            H = self.base_map.info.height
            resolution = self.base_map.info.resolution
            ox = self.base_map.info.origin.position.x
            oy = self.base_map.info.origin.position.y

            # 初始化為自由空間
            risk_map_data = np.zeros((H, W), dtype=np.uint8)
            
            # 為每個風險區域創建遮罩
            for polygon_points in risk_zones.markers:
                poly_px = []
                for pt in polygon_points.points:
                    x = int((pt.x - ox) / resolution)
                    y = int((pt.y - oy) / resolution)
                    # 確保坐標在地圖範圍內
                    x = max(0, min(x, W-1))
                    y = max(0, min(y, H-1))
                    poly_px.append([x, y])
                
                if len(poly_px) >= 3:  # 至少需要3個點形成多邊形
                    poly_px = np.array([poly_px], dtype=np.int32)
                    # 創建臨時遮罩
                    temp_mask = np.zeros((H, W), dtype=np.uint8)
                    cv2.fillPoly(temp_mask, [poly_px], 1)
                    # 將風險區域標記為障礙物 (100)
                    risk_map_data = np.where(temp_mask == 1, 100, risk_map_data)
            
            # 將numpy數組轉換為ROS消息格式並賦值給data字段
            risk_map.data = risk_map_data.flatten().tolist()
            
            return risk_map
            
        except Exception as e:
            self.get_logger().error(f"生成風險地圖時發生錯誤: {e}")
            return None


    def create_free_space_srv(self, req, res):
        self.get_logger().info("(service)create_free_space_srv call")
        
        # 立即返回接受狀態，然後異步處理
        res.success = True
        res.message = "開始創建自由空間，請稍候..."
        
        # 啟動異步處理
        self._start_create_free_space_async()
        
        return res
    
    def _start_create_free_space_async(self):
        """異步創建自由空間"""
        try:
            # 等待記錄區域列表服務可用
            if not self.get_record_zone_list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("記錄區域列表服務不可用")
                return
            
            # 調用獲取記錄區域列表服務
            zone_req = GetZoneList.Request()
            future = self.get_record_zone_list_client.call_async(zone_req)
            
            # 添加完成回調
            future.add_done_callback(self._handle_zone_list_response)
            
        except Exception as e:
            self.get_logger().error(f"啟動異步創建自由空間時發生錯誤: {e}")
    
    def _handle_zone_list_response(self, future):
        """處理區域列表服務響應"""
        try:
            if not future.done():
                self.get_logger().error("獲取記錄區域列表超時")
                return
            
            zone_response = future.result()
            
            if not zone_response.success:
                self.get_logger().error(f"獲取記錄區域列表失敗: {zone_response.message}")
                return

            # 為每個zone生成對應的mask和ZoneMap對象
            overall_freespace_map = self._create_zone_maps_and_freespace(
                zone_response.zone_list
            )
            self.base_map = overall_freespace_map
            if overall_freespace_map is not None:
                # 使用可靠QoS發布整體freespace地圖
                self.free_space_pub.publish(overall_freespace_map)
                self.get_logger().info(f"成功創建自由空間，包含 {len(self.zone_map_list)} 個區域")
                self.get_logger().info(f"成功創建 {len(self.zone_map_list)} 個ZoneMap對象")
            else:
                self.get_logger().error("生成自由空間失敗")
            
        except Exception as e:
            self.get_logger().error(f"處理區域列表響應時發生錯誤: {e}")


#===================
#建立all zone區
#freespace zone mask_list
#===================
    def _create_zone_maps_and_freespace(self, zone_list):
        self.zone_map_list = []
        
        if not zone_list.markers:
            self.get_logger().warn("沒有區域數據")
            return None
            
        # 計算所有區域的邊界
        all_points = []
        for polygon_points in zone_list.markers:
            for pt in polygon_points.points:
                all_points.append((pt.x, pt.y))
        
        if not all_points:
            self.get_logger().warn("沒有有效的點數據")
            return None
            
        min_x = min(pt[0] for pt in all_points)
        max_x = max(pt[0] for pt in all_points)
        min_y = min(pt[1] for pt in all_points)
        max_y = max(pt[1] for pt in all_points)

        resolution = 0.05  # 5cm 解析度
        margin = 1.0  # 邊界裕度
        
        # 計算地圖尺寸
        map_width = max_x - min_x + 2 * margin
        map_height = max_y - min_y + 2 * margin
        W = int(map_width / resolution)
        H = int(map_height / resolution)


        # 設定地圖原點（左下角）
        ox = min_x - margin
        oy = min_y - margin

        mask = np.zeros((H, W), dtype=np.uint8)
        for polygon_points in zone_list.markers:
            poly_px = []
            for pt in polygon_points.points:
                x = int((pt.x - ox) / resolution)
                y = int((pt.y - oy) / resolution)
                poly_px.append([x, y])
            poly_px = np.array([poly_px], dtype=np.int32)
            cv2.fillPoly(mask, [poly_px], 1)

            zone_map = ZoneMap(polygon_points.id, polygon_points.points)
            zone_map.mask = mask
            self.zone_map_list.append(zone_map)
            self.get_logger().info(f"成功創建zone map，包含 {len(self.zone_map_list)} 個區域")

        # 建立地圖數據（polygon 內為自由空間，外為障礙）
        occ_masked = np.where(mask == 1, 0, 100)  # 遮罩內為自由空間(0)，外為障礙(100)

# 發布遮罩後的地圖
        masked_map = OccupancyGrid()
        masked_map.header.stamp = self.get_clock().now().to_msg()
        masked_map.header.frame_id = 'map'
        masked_map.info.resolution = resolution
        masked_map.info.width = W
        masked_map.info.height = H
        masked_map.info.origin.position.x = ox
        masked_map.info.origin.position.y = oy
        masked_map.info.origin.position.z = 0.0
        masked_map.info.origin.orientation.x = 0.0
        masked_map.info.origin.orientation.y = 0.0
        masked_map.info.origin.orientation.z = 0.0
        masked_map.info.origin.orientation.w = 1.0
        masked_map.data = occ_masked.flatten().tolist()
                
        return masked_map
    def get_zone_map_list(self):
        """獲取當前的zone map列表"""
        return self.zone_map_list

    def get_zone_map_by_id(self, zone_id):
        """根據zone_id獲取特定的ZoneMap對象"""
        for zone_map in self.zone_map_list:
            if zone_map.getZoneId() == zone_id:
                return zone_map
        return None

class ZoneMap:
    def __init__(self,zone_id,polygon_points_list):
        self.zone_id = zone_id
        self.polygon_points_list = polygon_points_list
        self.mask = OccupancyGrid()
    def getZoneId(self):
        return self.zone_id
    def getPolygonPointsList(self):
        return self.polygon_points_list
    def getMask(self):
        return self.mask

def main(args=None):
    rclpy.init(args=args)
    node = MapManage()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()