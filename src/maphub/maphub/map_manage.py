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
from boustrophedon_coverage_interfaces.srv import ZoneMapList
from boustrophedon_coverage_interfaces.msg import ZoneMap
import math
from path_record_interface.srv import ChennalPathList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class MapManage(Node):
    def __init__(self):
        super().__init__('map_manage')
        self.get_logger().info("map_manage init")
        self.declare_parameter('inflate_radius_m', 0.08)  
        self.set_parameters([
            Parameter('use_sim_time',Parameter.Type.BOOL,True)
        ])
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_service(Trigger, '/create_risk_map', self.create_risk_map_srv)
        self.create_service(Trigger, '/create_free_space', self.create_free_space_srv)

        # 添加新的服务
        self.create_service(Trigger, '/create_chennal_map', self.create_chennal_map_srv)
        # 添加新的服务
        self.create_service(ZoneMapList, '/get_zone_map_list_srv', self.get_zone_map_list_srv)
        # self.create_service(Trigger, '/create_zone_cell_decomposition', self.create_zone_cell_decomposition_srv)

        self.service_callback_group = ReentrantCallbackGroup()
        self.get_risk_zone_list_client = self.create_client(
            GetZoneList, '/get_risk_zone_list',
            callback_group=self.service_callback_group
        )
        self.get_record_zone_list_client = self.create_client(
            GetZoneList, '/get_record_zone_list',
            callback_group=self.service_callback_group
        )
        
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

         #發布區
        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', qos)
        self.free_space_inflated_pub = self.create_publisher(OccupancyGrid, '/free_space_inflated', qos)
        self.risk_map_pub = self.create_publisher(OccupancyGrid, '/risk_map', qos)
        self.risk_map_inflated_pub = self.create_publisher(OccupancyGrid, '/risk_map_inflated', qos)

        self.latest_map = None #

        self.zone_list = []
        self.base_map = None

        # 添加 chennal path 客户端
        self.get_chennal_path_list_client = self.create_client(
            ChennalPathList, '/get_chennal_path_list',
            callback_group=self.service_callback_group
        )
        
        # 添加 chennal map 发布者
        self.chennal_map_pub = self.create_publisher(OccupancyGrid, '/chennal_map', qos)
        self.chennal_map_inflated_pub = self.create_publisher(OccupancyGrid, '/chennal_map_inflated', qos)
        
        # # 添加可视化发布者
        # self.cell_decomposition_lines_pub = self.create_publisher(MarkerArray, '/zone_cell_decomposition_lines', qos)
        # self.zone_cell_map_pub = self.create_publisher(OccupancyGrid, '/zone_cell_map', qos)
        
        # 添加参数
        self.declare_parameter('chennal_width_m', 0.6)  # 通道宽度（米）
        # self.declare_parameter('cell_width_m', 0.3)  # 细胞宽度（米）
        
        # # 存储zone cell maps
        # self.zone_cell_maps = []

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
        """處理風險區域列表服務響應/發布風險膨脹地圖"""
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
            risk_map_inflated = self._create_risk_map_inflated(risk_map)
            if risk_map is not None:
                self.risk_map_pub.publish(risk_map)
                self.risk_map_inflated_pub.publish(risk_map_inflated)
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

    def _create_risk_map_inflated(self, risk_map: OccupancyGrid):
        """
        膨脹風險地圖，使riskmask向外膨脹 inflate_r_m。
        """
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        resolution = risk_map.info.resolution
        H = risk_map.info.height
        W = risk_map.info.width
        risk_map_data = np.asarray(risk_map.data, dtype=np.int16).reshape(H, W)
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))
        # 將風險區域(100)膨脹、外擴
        if r_cells > 0:
            # 將風險點二值化 (1:風險, 0:非風險)
            risk_binary = (risk_map_data == 100).astype(np.uint8)
            # 用形態學膨脹
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            risk_binary_inflated = cv2.dilate(risk_binary, k)
            # 重新構造 risk_map 結果
            inflated_data = np.where(risk_binary_inflated == 1, 100, 0).astype(np.int8)
            inflated_map = OccupancyGrid()
            inflated_map.header = risk_map.header
            inflated_map.info = risk_map.info
            inflated_map.data = inflated_data.flatten().tolist()
            return inflated_map
        else:
            return risk_map

    def create_free_space_srv(self, req, res):
        self.get_logger().info("(service)create_chennal_free_space_srv call")
        
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
        # try:
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

        overall_freespace_map_inflated = self._create_free_space_inflated(overall_freespace_map)
        self.base_map = overall_freespace_map
        if overall_freespace_map is not None:
            # 使用可靠QoS發布整體freespace地圖
            self.free_space_pub.publish(overall_freespace_map)
            self.free_space_inflated_pub.publish(overall_freespace_map_inflated)
            self.get_logger().info(f"成功創建自由空間，包含 {len(self.zone_map_list)} 個區域")
            self.get_logger().info(f"成功創建 {len(self.zone_map_list)} 個ZoneMap對象")
        else:
            self.get_logger().error("生成自由空間失敗")
            
        # except Exception as e:
        #     self.get_logger().error(f"處理區域列表響應時發生錯誤: {e}")


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

        mask = np.zeros((H, W), dtype=np.uint8)
        for polygon_points in zone_list.markers:
            zone_mask = np.zeros((H, W), dtype=np.uint8)
            poly_px = []
            for pt in polygon_points.points:
                x = int((pt.x - ox) / resolution)
                y = int((pt.y - oy) / resolution)
                poly_px.append([x, y])
            poly_px = np.array([poly_px], dtype=np.int32)
            cv2.fillPoly(zone_mask, [poly_px], 1)
            cv2.fillPoly(mask, [poly_px], 1) #全局的MASK
            zone_occ_masked = np.where(zone_mask == 1, 0, 100)
            zone_map = ZoneMap()
            zone_map.zone_id = polygon_points.id
            zone_map.mask_map = masked_map
            zone_map.mask_map.data = zone_occ_masked.flatten().tolist()

            zone_map.mask_map_inflated = self._create_free_space_inflated(zone_map.mask_map)
            self.zone_map_list.append(zone_map)
            self.get_logger().info(f"成功創建zone map，包含 {len(self.zone_map_list)} 個區域")

        # 建立地圖數據（polygon 內為自由空間，外為障礙）
        occ_masked = np.where(mask == 1, 0, 100)  # 遮罩內為自由空間(0)，外為障礙(100)

# 發布遮罩後的地圖
        
        masked_map.data = occ_masked.flatten().tolist()
                
        return masked_map
    def _create_free_space_inflated(self, free_space_map: OccupancyGrid) -> OccupancyGrid:
        """
        向內膨脹自由空間地圖（將free_space_map '收縮' inflate_r_m），通常是使自由空間更為保守。
        """
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        resolution = free_space_map.info.resolution
        H = free_space_map.info.height
        W = free_space_map.info.width
        free_space_map_data = np.asarray(free_space_map.data, dtype=np.int16).reshape(H, W)
        # 自由空間為0，障礙為100
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))
        # 只處理膨脹距離大於0情況
        if r_cells > 0:
            # 生成膨脹內核
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            # 建立自由區域遮罩，free=1, 其他=0
            free_mask = (free_space_map_data == 0).astype(np.uint8)
            # 對自由區域做erode（向內膨脹/收縮），邊界會縮小
            eroded_free_mask = cv2.erode(free_mask, k)
            # 製作新地圖資料（erode後的自由區設為0，其餘設100）
            inflated_data = np.where(eroded_free_mask == 1, 0, 100).astype(np.int8)
            inflated_map = OccupancyGrid()
            inflated_map.header = free_space_map.header
            inflated_map.info = free_space_map.info
            inflated_map.data = inflated_data.flatten().tolist()
            return inflated_map
        else:
            return free_space_map

    def create_chennal_map_srv(self, req, res):
        """创建通道地图服务"""
        self.get_logger().info("(service)create_chennal_map_srv call")
        
        # 立即返回接受状态，然后异步处理
        res.success = True
        res.message = "开始创建通道地图，请稍候..."
        
        # 启动异步处理
        self._start_create_chennal_map_async()
        
        return res
    
    def _start_create_chennal_map_async(self):
        """异步创建通道地图"""
        try:
            # 等待 chennal path 列表服务可用
            if not self.get_chennal_path_list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("通道路径列表服务不可用")
                return
            
            # 调用获取 chennal path 列表服务
            chennal_req = ChennalPathList.Request()
            future = self.get_chennal_path_list_client.call_async(chennal_req)
            
            # 添加完成回调
            future.add_done_callback(self._handle_chennal_path_list_response)
            
        except Exception as e:
            self.get_logger().error(f"启动异步创建通道地图时发生错误: {e}")

    def _handle_chennal_path_list_response(self, future):
        """处理通道路径列表服务响应"""
        try:
            if not future.done():
                self.get_logger().error("获取通道路径列表超时")
                return
            
            chennal_response = future.result()
            
            if not chennal_response.success:
                self.get_logger().error(f"获取通道路径列表失败: {chennal_response.message}")
                return
            
            # 生成通道地图
            chennal_map = self._generate_chennal_map(chennal_response.chennal_path_array)
            chennal_map_inflated = self._create_chennal_map_inflated(chennal_map)
            
            if chennal_map is not None:
                self.chennal_map_pub.publish(chennal_map)
                self.chennal_map_inflated_pub.publish(chennal_map_inflated)
                self.get_logger().info(f"成功创建通道地图，包含 {len(chennal_response.chennal_path_array.markers)} 条通道")
            else:
                self.get_logger().error("生成通道地图失败")
            
        except Exception as e:
            self.get_logger().error(f"处理通道路径列表响应时发生错误: {e}")

    def _generate_chennal_map(self, chennal_path_array):
        """根据通道路径生成通道地图"""
        try:
            if not chennal_path_array.markers:
                self.get_logger().warn("没有通道路径数据")
                return None
            if not self.base_map:
                self.get_logger().error("没有基础地图")
                return None
            # 计算所有路径点的边界
            all_points = []
            for marker in chennal_path_array.markers:
                for point in marker.points:
                    all_points.append((point.x, point.y))
            
            if not all_points:
                self.get_logger().warn("没有有效的路径点数据")
                return None
            
            chennal_width = float(self.get_parameter('chennal_width_m').value)

            # 创建通道地图
            chennal_map = OccupancyGrid()
            chennal_map.header.stamp = self.get_clock().now().to_msg()
            chennal_map.header.frame_id = self.base_map.header.frame_id
            chennal_map.info = self.base_map.info
            H = self.base_map.info.height
            W = self.base_map.info.width
            ox = self.base_map.info.origin.position.x
            oy = self.base_map.info.origin.position.y
            
            # 初始化为障碍物
            chennal_map_data = np.full((H, W), 100, dtype=np.uint8)
            resolution = self.base_map.info.resolution
     
            # 为每条路径创建通道
            for marker in chennal_path_array.markers:
                if len(marker.points) < 2:
                    continue
                
                # 将路径点转换为像素坐标
                path_points = []
                for point in marker.points:
                    x = int((point.x - ox) / resolution)
                    y = int((point.y - oy) / resolution)
                    # 确保坐标在地图范围内
                    x = max(0, min(x, W-1))
                    y = max(0, min(y, H-1))
                    path_points.append((x, y))
                
                # 为路径线段创建膨胀的通道
                chennal_width_pixels = int(chennal_width / resolution)
                
                for i in range(len(path_points) - 1):
                    # 在两点之间画线，并膨胀
                    pt1 = path_points[i]
                    pt2 = path_points[i + 1]
                    
                    # 创建临时图像来画线
                    temp_img = np.zeros((H, W), dtype=np.uint8)
                    cv2.line(temp_img, pt1, pt2, 1, thickness=chennal_width_pixels)
                    
                    # 将通道区域标记为自由空间
                    chennal_map_data = np.where(temp_img == 1, 0, chennal_map_data)
            
            # 将numpy数组转换为ROS消息格式
            chennal_map.data = chennal_map_data.flatten().tolist()
            return chennal_map
            
        except Exception as e:
            self.get_logger().error(f"生成通道地图时发生错误: {e}")
            return None

    def _create_chennal_map_inflated(self, chennal_map: OccupancyGrid):
        """
        膨胀通道地图，使通道向外膨胀 inflate_r_m
        """
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        resolution = chennal_map.info.resolution
        H = chennal_map.info.height
        W = chennal_map.info.width
        chennal_map_data = np.asarray(chennal_map.data, dtype=np.int16).reshape(H, W)
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))
        
        # 将通道区域(0)向內膨胀（腐蚀处理）
        if r_cells > 0:
            # 通道二值化 (1:通道, 0:非通道)
            chennal_binary = (chennal_map_data == 0).astype(np.uint8)
            # 用形態學腐蝕（向內收縮通道區）
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            chennal_binary_eroded = cv2.erode(chennal_binary, k)
            # 重新构造 chennal_map 結果
            eroded_data = np.where(chennal_binary_eroded == 1, 0, 100).astype(np.int8)
            eroded_map = OccupancyGrid()
            eroded_map.header = chennal_map.header
            eroded_map.info = chennal_map.info
            eroded_map.data = eroded_data.flatten().tolist()
            return eroded_map
        else:
            return chennal_map

    # def create_zone_cell_decomposition_srv(self, req, res):
    #     """创建zone细胞分解服务"""
    #     self.get_logger().info("(service)create_zone_cell_decomposition_srv call")
        
    #     # 立即返回接受状态，然后异步处理
    #     res.success = True
    #     res.message = "开始创建zone细胞分解，请稍候..."
        
    #     # 启动异步处理
    #     self._start_create_zone_cell_decomposition_async()
        
    #     return res
    
    # def _start_create_zone_cell_decomposition_async(self):
    #     """异步创建zone细胞分解"""
    #     try:
    #         # 等待记录区域列表服务可用
    #         if not self.get_record_zone_list_client.wait_for_service(timeout_sec=5.0):
    #             self.get_logger().error("记录区域列表服务不可用")
    #             return
            
    #         # 调用获取记录区域列表服务
    #         zone_req = GetZoneList.Request()
    #         future = self.get_record_zone_list_client.call_async(zone_req)
            
    #         # 添加完成回调
    #         future.add_done_callback(self._handle_zone_cell_decomposition_response)
            
    #     except Exception as e:
    #         self.get_logger().error(f"启动异步创建zone细胞分解时发生错误: {e}")
    
    # def _handle_zone_cell_decomposition_response(self, future):
    #     """处理zone细胞分解响应"""
    #     try:
    #         if not future.done():
    #             self.get_logger().error("获取记录区域列表超时")
    #             return
            
    #         zone_response = future.result()
            
    #         if not zone_response.success:
    #             self.get_logger().error(f"获取记录区域列表失败: {zone_response.message}")
    #             return
            
    #         # 为每个zone进行细胞分解
    #         self.zone_cell_maps = []
    #         all_decomposition_lines = MarkerArray()
            
    #         for zone_idx, polygon_points in enumerate(zone_response.zone_list.markers):
    #             zone_cell_map, decomposition_lines = self._perform_boustrophedon_cellular_decomposition(
    #                 polygon_points, zone_idx
    #             )
                
    #             if zone_cell_map is not None:
    #                 self.zone_cell_maps.append({
    #                     'zone_id': polygon_points.id,
    #                     'cell_map': zone_cell_map,
    #                     'decomposition_lines': decomposition_lines
    #                 })
                    
    #                 # 添加到总的可视化标记中
    #                 all_decomposition_lines.markers.extend(decomposition_lines.markers)
            
    #         # 发布可视化线段
    #         if all_decomposition_lines.markers:
    #             self.cell_decomposition_lines_pub.publish(all_decomposition_lines)
    #             self.get_logger().info(f"成功创建 {len(self.zone_cell_maps)} 个zone的细胞分解")
            
    #         # 创建并发布合并的zone cell map
    #         combined_cell_map = self._create_combined_zone_cell_map()
    #         if combined_cell_map is not None:
    #             self.zone_cell_map_pub.publish(combined_cell_map)
                
    #     except Exception as e:
    #         self.get_logger().error(f"处理zone细胞分解响应时发生错误: {e}")

    # def _perform_boustrophedon_cellular_decomposition(self, polygon_points, zone_idx):
    #     """对单个zone执行boustrophedon细胞分解"""
    #     try:
    #         if not self.base_map:
    #             self.get_logger().error("没有基础地图进行细胞分解")
    #             return None, MarkerArray()
            
    #         # 获取基础地图信息
    #         H = self.base_map.info.height
    #         W = self.base_map.info.width
    #         resolution = self.base_map.info.resolution
    #         ox = self.base_map.info.origin.position.x
    #         oy = self.base_map.info.origin.position.y
            
    #         # 创建zone mask
    #         zone_mask = np.zeros((H, W), dtype=np.uint8)
    #         poly_px = []
    #         for pt in polygon_points.points:
    #             x = int((pt.x - ox) / resolution)
    #             y = int((pt.y - oy) / resolution)
    #             # 确保坐标在地图范围内
    #             x = max(0, min(x, W-1))
    #             y = max(0, min(y, H-1))
    #             poly_px.append([x, y])
            
    #         if len(poly_px) >= 3:
    #             poly_px = np.array([poly_px], dtype=np.int32)
    #             cv2.fillPoly(zone_mask, [poly_px], 1)
    #         else:
    #             return None, MarkerArray()
            
    #         # 执行boustrophedon细胞分解
    #         cell_width_m = float(self.get_parameter('cell_width_m').value)
    #         cell_width_pixels = max(1, int(cell_width_m / resolution))
            
    #         # 创建细胞地图
    #         cell_map = self._create_boustrophedon_cells(zone_mask, cell_width_pixels, H, W, resolution, ox, oy)
            
    #         # 创建可视化线段
    #         decomposition_lines = self._create_decomposition_visualization(
    #             zone_mask, cell_width_pixels, H, W, resolution, ox, oy, zone_idx
    #         )
            
    #         return cell_map, decomposition_lines
            
    #     except Exception as e:
    #         self.get_logger().error(f"执行zone {zone_idx} 细胞分解时发生错误: {e}")
    #         return None, MarkerArray()

    # def _create_boustrophedon_cells(self, zone_mask, cell_width_pixels, H, W, resolution, ox, oy):
    #     """创建boustrophedon细胞地图"""
    #     try:
    #         # 创建细胞地图
    #         cell_map = OccupancyGrid()
    #         cell_map.header.stamp = self.get_clock().now().to_msg()
    #         cell_map.header.frame_id = 'map'
    #         cell_map.info = self.base_map.info
            
    #         # 初始化为障碍物
    #         cell_data = np.full((H, W), 100, dtype=np.uint8)
            
    #         # 找到zone的边界
    #         zone_indices = np.where(zone_mask == 1)
    #         if len(zone_indices[0]) == 0:
    #             cell_map.data = cell_data.flatten().tolist()
    #             return cell_map
            
    #         min_col = np.min(zone_indices[1])
    #         max_col = np.max(zone_indices[1])
            
    #         # 按列进行boustrophedon分解
    #         cell_id = 1
    #         for col in range(min_col, max_col + 1, cell_width_pixels):
    #             # 找到该列中zone的连续段
    #             col_mask = zone_mask[:, col] if col < W else np.zeros(H)
    #             segments = self._find_continuous_segments(col_mask)
                
    #             for start_row, end_row in segments:
    #                 # 为每个连续段创建一个细胞
    #                 cell_start_col = col
    #                 cell_end_col = min(col + cell_width_pixels, W)
                    
    #                 # 在细胞区域内标记为自由空间，并给每个细胞一个唯一ID
    #                 for c in range(cell_start_col, cell_end_col):
    #                     for r in range(start_row, end_row + 1):
    #                         if c < W and r < H and zone_mask[r, c] == 1:
    #                             cell_data[r, c] = 0  # 自由空间
                    
    #                 cell_id += 1
            
    #         cell_map.data = cell_data.flatten().tolist()
    #         return cell_map
            
    #     except Exception as e:
    #         self.get_logger().error(f"创建boustrophedon细胞时发生错误: {e}")
    #         return None

    # def _find_continuous_segments(self, col_mask):
    #     """找到列中的连续段"""
    #     segments = []
    #     start = None
        
    #     for i, val in enumerate(col_mask):
    #         if val == 1 and start is None:
    #             start = i
    #         elif val == 0 and start is not None:
    #             segments.append((start, i - 1))
    #             start = None
        
    #     # 处理最后一个段
    #     if start is not None:
    #         segments.append((start, len(col_mask) - 1))
        
    #     return segments

    # def _create_decomposition_visualization(self, zone_mask, cell_width_pixels, H, W, resolution, ox, oy, zone_idx):
    #     """创建分解可视化线段"""
    #     try:
    #         marker_array = MarkerArray()
            
    #         # 找到zone的边界
    #         zone_indices = np.where(zone_mask == 1)
    #         if len(zone_indices[0]) == 0:
    #             return marker_array
            
    #         min_col = np.min(zone_indices[1])
    #         max_col = np.max(zone_indices[1])
    #         min_row = np.min(zone_indices[0])
    #         max_row = np.max(zone_indices[0])
            
    #         marker_id = zone_idx * 1000  # 为每个zone分配不同的ID范围
            
    #         # 创建垂直分割线
    #         for col in range(min_col, max_col + 1, cell_width_pixels):
    #             if col >= W:
    #                 continue
                    
    #             # 找到该列的zone范围
    #             col_mask = zone_mask[:, col]
    #             segments = self._find_continuous_segments(col_mask)
                
    #             for start_row, end_row in segments:
    #                 # 创建垂直线标记
    #                 line_marker = Marker()
    #                 line_marker.header.frame_id = 'map'
    #                 line_marker.header.stamp = self.get_clock().now().to_msg()
    #                 line_marker.ns = f"zone_{zone_idx}_vertical_lines"
    #                 line_marker.id = marker_id
    #                 line_marker.type = Marker.LINE_STRIP
    #                 line_marker.action = Marker.ADD
                    
    #                 # 设置线条属性
    #                 line_marker.scale.x = 0.02  # 线条宽度
    #                 line_marker.color.r = 1.0
    #                 line_marker.color.g = 0.0
    #                 line_marker.color.b = 0.0
    #                 line_marker.color.a = 0.8
                    
    #                 # 添加线段端点
    #                 start_point = Point()
    #                 start_point.x = ox + (col + 0.5) * resolution
    #                 start_point.y = oy + (start_row + 0.5) * resolution
    #                 start_point.z = 0.1
                    
    #                 end_point = Point()
    #                 end_point.x = ox + (col + 0.5) * resolution
    #                 end_point.y = oy + (end_row + 0.5) * resolution
    #                 end_point.z = 0.1
                    
    #                 line_marker.points.append(start_point)
    #                 line_marker.points.append(end_point)
                    
    #                 marker_array.markers.append(line_marker)
    #                 marker_id += 1
            
    #         # 创建水平分割线（可选，用于显示细胞边界）
    #         for col in range(min_col, max_col + 1, cell_width_pixels):
    #             if col >= W:
    #                 continue
                    
    #             col_mask = zone_mask[:, col]
    #             segments = self._find_continuous_segments(col_mask)
                
    #             for start_row, end_row in segments:
    #                 # 在细胞的顶部和底部创建水平线
    #                 for row in [start_row, end_row]:
    #                     line_marker = Marker()
    #                     line_marker.header.frame_id = 'map'
    #                     line_marker.header.stamp = self.get_clock().now().to_msg()
    #                     line_marker.ns = f"zone_{zone_idx}_horizontal_lines"
    #                     line_marker.id = marker_id
    #                     line_marker.type = Marker.LINE_STRIP
    #                     line_marker.action = Marker.ADD
                        
    #                     # 设置线条属性
    #                     line_marker.scale.x = 0.02
    #                     line_marker.color.r = 0.0
    #                     line_marker.color.g = 1.0
    #                     line_marker.color.b = 0.0
    #                     line_marker.color.a = 0.6
                        
    #                     # 添加水平线段
    #                     start_point = Point()
    #                     start_point.x = ox + (col + 0.5) * resolution
    #                     start_point.y = oy + (row + 0.5) * resolution
    #                     start_point.z = 0.1
                        
    #                     end_point = Point()
    #                     end_point.x = ox + (min(col + cell_width_pixels, W) - 0.5) * resolution
    #                     end_point.y = oy + (row + 0.5) * resolution
    #                     end_point.z = 0.1
                        
    #                     line_marker.points.append(start_point)
    #                     line_marker.points.append(end_point)
                        
    #                     marker_array.markers.append(line_marker)
    #                     marker_id += 1
            
    #         return marker_array
            
    #     except Exception as e:
    #         self.get_logger().error(f"创建分解可视化时发生错误: {e}")
    #         return MarkerArray()

    # def _create_combined_zone_cell_map(self):
    #     """创建合并的zone细胞地图"""
    #     try:
    #         if not self.zone_cell_maps or not self.base_map:
    #             return None
            
    #         # 创建合并地图
    #         combined_map = OccupancyGrid()
    #         combined_map.header.stamp = self.get_clock().now().to_msg()
    #         combined_map.header.frame_id = 'map'
    #         combined_map.info = self.base_map.info
            
    #         H = self.base_map.info.height
    #         W = self.base_map.info.width
            
    #         # 初始化为障碍物
    #         combined_data = np.full((H, W), 100, dtype=np.uint8)
            
    #         # 合并所有zone的细胞地图
    #         for zone_cell_info in self.zone_cell_maps:
    #             cell_map = zone_cell_info['cell_map']
    #             cell_data = np.asarray(cell_map.data, dtype=np.uint8).reshape(H, W)
                
    #             # 将自由空间合并到总地图中
    #             combined_data = np.where(cell_data == 0, 0, combined_data)
            
    #         combined_map.data = combined_data.flatten().tolist()
    #         return combined_map
            
    #     except Exception as e:
    #         self.get_logger().error(f"创建合并zone细胞地图时发生错误: {e}")
    #         return None

    def get_zone_map_list_srv(self, req, res):
        """獲取當前的zone map列表"""
        res.zone_map_list = self.zone_map_list
        return res

    def get_zone_cell_maps(self):
        """获取zone细胞地图列表"""
        return self.zone_cell_maps


# class ZoneMap:
#     def __init__(self):
#         self.zone_id = None
#         # self.polygon_points_list = None
#         self.mask_map = OccupancyGrid()
#         self.mask_map_inflated = OccupancyGrid()
#     def getZoneId(self):
#         return self.zone_id
#     def getPolygonPointsList(self):
#         return self.polygon_points_list
#     def getMask(self):
#         return self.mask_map 
#     def getMaskInflated(self):
#         return self.mask_map_inflated

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