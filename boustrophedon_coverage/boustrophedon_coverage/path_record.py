
import rclpy
from rclpy.node import Node
from nav_msgs.msg import  Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import Buffer, TransformListener
import json 
import math, os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from boustrophedon_coverage.path_record_utils import *
from boustrophedon_coverage_interfaces.srv import GetZoneList

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_dist', 0.05)   # 最小移動距離(公尺)才記錄
        self.declare_parameter('min_dt', 0.10)     # 最小時間間隔(秒)才記錄
        self.declare_parameter('frame_id', 'map')  # Path frame
        self.declare_parameter('save_dir', 'zone_record')  # 保存區域列表的目錄
        self.declare_parameter('polygon_simplify_dist', 0.1)  # 多邊形簡化距離
        self.get_logger().info('path_recorder ready.')

        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)  


        # 設置與 boustrophedon_coverage 相同的 QoS
        polygon_qos = QoSProfile(depth=1)
        polygon_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        polygon_qos.reliability = QoSReliabilityPolicy.RELIABLE


        # 使用 QoS 創建多邊形發布者
        # 新增區域點標記發布者
        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.zone_marker_pub = self.create_publisher(Marker, '/zone_markers', polygon_qos)
        self.zone_list_pub = self.create_publisher(MarkerArray,'/zone_list', polygon_qos)

        # 風險區域發布者
        self.risk_zone_marker_pub = self.create_publisher(Marker, '/risk_zone_markers', polygon_qos)
        self.risk_zone_list_pub = self.create_publisher(MarkerArray, '/risk_zone_list', polygon_qos)


        # 新增一個定時器，定時發布path，確保即使鍵盤遙控時也能看到path
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.publish_path_timer)

        # 現有服務
        # self.create_service(SetBool, '/record_path_status', self.record_path_status_srv)
        # 新增區域記錄服務
        self.create_service(Trigger, '/risk_zone_start', self.risk_zone_start_srv)
        self.create_service(Trigger, '/risk_zone_end', self.risk_zone_end_srv)
        self.create_service(Trigger, '/save_zone_list', self.save_zone_list_srv)
        self.create_service(Trigger, '/load_zone_list', self.load_zone_list_srv)

        self.create_service(Trigger, '/record_zone_start', self.record_zone_start_srv)
        self.create_service(Trigger, '/record_zone_end', self.record_zone_end_srv)
        
        # 新增獲取區域列表的服務
        self.create_service(Trigger, '/get_record_zone_info', self.get_record_zone_info_srv)
        self.create_service(GetZoneList, '/get_record_zone_list', self.get_record_zone_list_srv)
        self.create_service(GetZoneList, '/get_risk_zone_list', self.get_risk_zone_list_srv)


        # 新增風險區域相關的路徑記錄
        self.risk_path = Path()
        self.risk_path.header.frame_id = self.get_parameter('frame_id').value
        

        self.record_zone_status = False
        self.record_zone_id = 0
        self.record_zone_name = "zone_" + str(self.record_zone_id)
        self.record_zone_marker = Marker()  # 記錄當前的zone point
        self.record_zone_list = MarkerArray()  # 全域zone list
        
        
        self.risk_zone_status = False
        self.risk_zone_id = 0
        self.risk_zone_name = "risk_zone_" + str(self.risk_zone_id)
        self.risk_zone_marker = Marker()  # 記錄當前的risk zone point
        self.risk_zone_list = MarkerArray()  # 全域risk zone list

        # 初始化路徑
        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value
        self.last_pt = None
        self.last_t = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 改進初始化：設置為None，讓第一次獲取成功時自動初始化
        self.last_robot_pos = None
        self.initialized = False
        
        # 添加初始化定時器，等待TF可用
        self.init_timer = self.create_timer(0.5, self.try_initialize)
        
        # 在初始化完成後自動載入區域資料
        # self.create_timer(1.0, self.auto_load_zones_on_startup)  # 延遲1秒載入
        
    # ============================================================
    # 嘗試初始化機器人位置
    # ============================================================
    def try_initialize(self):
        #===========
        # 初始化Makerarray
        #===========
        def publish_empty_markerarrays():
            """清除 RViz 上的 zone_list 與 risk_zone_list"""
            frame = self.get_parameter('frame_id').value
            now = self.get_clock().now().to_msg()

            # 清除 zone_markers
            delete_zone_marker = Marker()
            delete_zone_marker.header.frame_id = frame
            delete_zone_marker.header.stamp = now
            delete_zone_marker.ns = "zones"
            delete_zone_marker.action = Marker.DELETEALL
            self.zone_marker_pub.publish(delete_zone_marker)

            # 清除 risk_zone_markers
            delete_risk_marker = Marker()
            delete_risk_marker.header.frame_id = frame
            delete_risk_marker.header.stamp = now
            delete_risk_marker.ns = "risk_zones"
            delete_risk_marker.action = Marker.DELETEALL
            self.risk_zone_marker_pub.publish(delete_risk_marker)

            # 清空內部列表
            self.record_zone_list = MarkerArray()
            self.risk_zone_list = MarkerArray()

            self.get_logger().info("✅ 已清空 RViz zone_markers 與 risk_zone_markers")


        if not self.initialized:
            robot_pos = self.get_robot_pos()
            if robot_pos is not None:
                self.last_robot_pos = robot_pos
                self.initialized = True

                publish_empty_markerarrays()

                # 發佈空 markerarray
                self.zone_list_pub.publish(self.record_zone_list)
                self.risk_zone_list_pub.publish(self.risk_zone_list)

                self.get_logger().info(f"機器人位置初始化成功: x={robot_pos.pose.position.x:.3f}, y={robot_pos.pose.position.y:.3f}")
                self.path.header.frame_id = self.get_parameter('frame_id').value
                self.path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(self.path)
                self.init_timer.cancel()
            else:
                self.get_logger().warn("等待TF可用以初始化機器人位置...")
   
    # ============================================================
    # 獲取機器人位置
    # ============================================================
    def get_robot_pos(self):
        """
        讀取tf，將odom座標轉換成目標map座標系下的位置
        回傳(x, y, yaw)，若失敗則回傳None
        """
        try:
            # 取得當前時間
            now = rclpy.time.Time()
            # 查詢從odom到map的轉換
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            # 取得平移
            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"TF查詢失敗: {e}")
            return None
            
    # ============================================================
    # 根據路徑創建多邊形
    # ============================================================
    def create_polygon_from_path(self, poses):
        if len(poses) < 3:
            return None
        simplify_dist = self.get_parameter('polygon_simplify_dist').value

        # 簡化路徑以減少多邊形頂點數量
        simplified_poses = simplify_path(poses, simplify_dist)
        
        # 創建多邊形marker
        marker = Marker()
        marker.header.frame_id = self.path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "zones"
        marker.id = self.record_zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 線條寬度
        marker.color.a = 0.5   # 透明度
        marker.color.r = 0.6
        marker.color.g = 0.0
        marker.color.b = 1.0   # 藍色
        
        # 添加所有點
        marker.points = []
        for pose in simplified_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            marker.points.append(point)
            
        # 如果路徑是封閉的（起點和終點接近），則添加起點作為最後一個點形成封閉多邊形
        if len(simplified_poses) > 2:
            first_pt = simplified_poses[0].pose.position
            last_pt = simplified_poses[-1].pose.position
            dist = math.sqrt((first_pt.x - last_pt.x)**2 + (first_pt.y - last_pt.y)**2)
            
            # 如果起點和終點距離較近，認為是封閉路徑
            if dist < 1.0:  # 1米內認為是封閉的
                point = Point()
                point.x = first_pt.x
                point.y = first_pt.y
                point.z = 0.0
                marker.points.append(point)
        
        return marker
        
    # ============================================================
    # 為風險區域創建多邊形，使用不同的顏色
    # ============================================================
    def create_risk_polygon_from_path(self, poses):
        """為風險區域創建多邊形，使用不同的顏色"""
        if len(poses) < 3:
            return None
        simplify_dist = self.get_parameter('polygon_simplify_dist').value

        # 簡化路徑以減少多邊形頂點數量
        simplified_poses = simplify_path(poses, simplify_dist)
        
        # 創建多邊形marker
        marker = Marker()
        marker.header.frame_id = self.risk_path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "risk_zones"
        marker.id = self.risk_zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # 稍微粗一點的線條
        marker.color.a = 0.8   # 更不透明
        marker.color.r = 1.0   # 紅色表示風險區域
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        # 添加所有點
        marker.points = []
        for pose in simplified_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            marker.points.append(point)
            
        # 如果路徑是封閉的（起點和終點接近），則添加起點作為最後一個點形成封閉多邊形
        if len(simplified_poses) > 2:
            first_pt = simplified_poses[0].pose.position
            last_pt = simplified_poses[-1].pose.position
            dist = math.sqrt((first_pt.x - last_pt.x)**2 + (first_pt.y - last_pt.y)**2)
            
            # 如果起點和終點距離較近，認為是封閉路徑
            if dist < 1.0:  # 1米內認為是封閉的
                point = Point()
                point.x = first_pt.x
                point.y = first_pt.y
                point.z = 0.0
                marker.points.append(point)
        
        return marker
        
    # ============================================================
    # 定時發布trace path
    # 功能： 發佈機器人路徑
    # ============================================================
    def publish_path_timer(self):
        # 原有的普通區域記錄邏輯
        if self.record_zone_status:
            # 如果還沒初始化，先嘗試初始化
            if not self.initialized:
                return
                
            robot_pos = self.get_robot_pos()
            now = self.get_clock().now()
            
            # 如果當前位置獲取失敗，跳過這次記錄
            if robot_pos is None:
                return
            # print(robot_pos.pose.position.x, robot_pos.pose.position.y)
            dt = (now - rclpy.time.Time.from_msg(self.last_robot_pos.header.stamp)).nanoseconds * 1e-9
            dx = robot_pos.pose.position.x - self.last_robot_pos.pose.position.x
            dy = robot_pos.pose.position.y - self.last_robot_pos.pose.position.y

            if math.hypot(dx, dy) >= self.get_parameter('min_dist').value and dt >= self.get_parameter('min_dt').value:
                self.path.poses.append(robot_pos)
                self.last_pt = robot_pos.pose.position
                self.last_robot_pos = robot_pos
                self.path.header.stamp = self.get_clock().now().to_msg()

                self.path_pub.publish(self.path)
                
                # 發布多邊形
                if len(self.path.poses) >= 3:
                    self.record_zone_marker = self.create_polygon_from_path(self.path.poses)
                    if self.record_zone_marker is not None:
                        self.zone_marker_pub.publish(self.record_zone_marker)

        # 新增風險區域記錄邏輯
        if self.risk_zone_status:
            if not self.initialized:
                return
                
            robot_pos = self.get_robot_pos()
            now = self.get_clock().now()
            
            if robot_pos is None:
                return
                
            # 如果是第一次記錄風險區域，初始化last_robot_pos
            if self.last_robot_pos is None:
                self.last_robot_pos = robot_pos
                return
                
            dt = (now - rclpy.time.Time.from_msg(self.last_robot_pos.header.stamp)).nanoseconds * 1e-9
            dx = robot_pos.pose.position.x - self.last_robot_pos.pose.position.x
            dy = robot_pos.pose.position.y - self.last_robot_pos.pose.position.y

            if math.hypot(dx, dy) >= self.get_parameter('min_dist').value and dt >= self.get_parameter('min_dt').value:
                self.risk_path.poses.append(robot_pos)
                self.last_robot_pos = robot_pos
                self.risk_path.header.stamp = self.get_clock().now().to_msg()
                
                # 發布風險區域多邊形
                if len(self.risk_path.poses) >= 3:
                    self.risk_zone_marker = self.create_risk_polygon_from_path(self.risk_path.poses)
                    if self.risk_zone_marker is not None:
                        self.risk_zone_marker_pub.publish(self.risk_zone_marker)

    # ============================================================
    # 記錄區域起始點 新增zone 區域
    # ============================================================
    def record_zone_start_srv(self, req, res):
        # 檢查是否正在記錄風險區域
        if self.risk_zone_status:
            self.get_logger().warn("無法開始記錄普通區域：目前正在記錄風險區域")
            res.success = False
            res.message = "無法開始記錄普通區域：目前正在記錄風險區域，請先結束風險區域記錄"
            return res
            
        self.get_logger().info("記錄區域起始點")
        self.record_zone_status = True
        self.record_zone_id += 1
        self.record_zone_name = "zone_" + str(self.record_zone_id)

        # 清空路徑，開始記錄新的區域
        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value

        # 重新初始化機器人位置，確保第二個zone可以正常開始記錄
        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos
            self.get_logger().info(f"重新初始化機器人位置: x={robot_pos.pose.position.x:.3f}, y={robot_pos.pose.position.y:.3f}")

        self.record_zone_marker = Marker()
        self.zone_marker_pub.publish(self.record_zone_marker)  # 刷新zone marker

        res.success = True
        res.message = "成功記錄區域起始點"
        return res

    # ============================================================
    # 記錄區域結束點
    # ============================================================
    def record_zone_end_srv(self, req, res):
        self.get_logger().info("記錄區域結束點")
        self.record_zone_status = False
        self.record_zone_list.markers.append(self.record_zone_marker)
        self.record_zone_marker = Marker()

        self.zone_list_pub.publish(self.record_zone_list)  # 發佈全域zone list

        res.success = True
        res.message = f"成功記錄區域結束點 #{len(self.record_zone_list.markers)}"
        return res

    # ============================================================
    # 儲存區域列表服務
    # ============================================================
    def save_zone_list_srv(self, req, res):
        """合并的区域列表保存服务 - 同时保存普通区域和风险区域"""
        success_count = 0
        error_messages = []
        
        # 保存普通区域列表
        if self._save_zone_list():
            success_count += 1
            self.get_logger().info("成功储存普通区域列表")
        else:
            error_messages.append("储存普通区域列表失败")
        
        # 保存风险区域列表
        if self._save_risk_zone_list():
            success_count += 1
            self.get_logger().info("成功储存风险区域列表")
        else:
            error_messages.append("储存风险区域列表失败")
        
        # 根据结果设置响应
        if success_count == 2:
            res.success = True
            res.message = "成功储存所有区域列表（普通区域 + 风险区域）"
        elif success_count == 1:
            res.success = True
            res.message = f"部分成功储存区域列表。错误: {'; '.join(error_messages)}"
        else:
            res.success = False
            res.message = f"储存区域列表失败: {'; '.join(error_messages)}"
        
        return res

    # ============================================================
    # 載入區域列表服務
    # ============================================================
    def load_zone_list_srv(self, req, res):
        """合并的区域列表加载服务 - 同时加载普通区域和风险区域并发布"""
        success_count = 0
        error_messages = []
        
        # 加载普通区域列表
        if self._load_zone_list():
            success_count += 1
            self.get_logger().info(f"成功載入普通區域列表: {len(self.record_zone_list.markers)} 个区域")
            self.zone_list_pub.publish(self.record_zone_list)  # 发布普通区域
        else:
            error_messages.append("載入普通區域列表失敗")
        
        # 加载风险区域列表
        if self._load_risk_zone_list():
            success_count += 1
            self.get_logger().info(f"成功載入風險區域列表: {len(self.risk_zone_list.markers)} 个风险区域")
            self.risk_zone_list_pub.publish(self.risk_zone_list)  # 发布风险区域
        else:
            error_messages.append("載入風險區域列表失敗")
        
        # 根据结果设置响应
        if success_count == 2:
            res.success = True
            res.message = "成功載入所有區域列表（普通區域 + 風險區域）"
        elif success_count == 1:
            res.success = True
            res.message = f"部分成功載入區域列表。错误: {'; '.join(error_messages)}"
        else:
            res.success = False
            res.message = f"載入區域列表失敗: {'; '.join(error_messages)}"
        
        return res

    # ============================================================
    # 風險區域開始記錄服務
    # ============================================================
    def risk_zone_start_srv(self, req, res):
        # 檢查是否正在記錄普通區域
        if self.record_zone_status:
            self.get_logger().warn("無法開始記錄風險區域：目前正在記錄普通區域")
            res.success = False
            res.message = "無法開始記錄風險區域：目前正在記錄普通區域，請先結束普通區域記錄"
            return res
            
        self.get_logger().info("開始記錄風險區域")
        self.risk_zone_status = True
        self.risk_zone_id += 1
        self.risk_zone_name = "risk_zone_" + str(self.risk_zone_id)

        # 清空風險路徑，開始記錄新的風險區域
        self.risk_path = Path()
        self.risk_path.header.frame_id = self.get_parameter('frame_id').value

        # 重新初始化機器人位置
        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos
            self.get_logger().info(f"風險區域記錄 - 重新初始化機器人位置: x={robot_pos.pose.position.x:.3f}, y={robot_pos.pose.position.y:.3f}")

        self.risk_zone_marker = Marker()
        self.risk_zone_marker_pub.publish(self.risk_zone_marker)  # 刷新風險區域marker

        res.success = True
        res.message = "成功開始記錄風險區域"
        return res

    # ============================================================
    # 風險區域結束記錄服務
    # ============================================================
    def risk_zone_end_srv(self, req, res):
        self.get_logger().info("結束記錄風險區域")
        self.risk_zone_status = False
        
        # 將當前風險區域添加到列表中
        if hasattr(self, 'risk_zone_marker') and self.risk_zone_marker.points:
            self.risk_zone_list.markers.append(self.risk_zone_marker)
            self.risk_zone_list_pub.publish(self.risk_zone_list)  # 發布全域風險區域列表
        
        self.risk_zone_marker = Marker()
        
        res.success = True
        res.message = f"成功結束記錄風險區域 #{len(self.risk_zone_list.markers)}"
        return res

    # ============================================================
    # 風險區域儲存服務
    # ============================================================
    def risk_zone_save_srv(self, req, res):
        self.get_logger().info("儲存風險區域")
        if self._save_risk_zone_list():
            res.success = True
            res.message = "成功儲存風險區域"
        else:
            res.success = False
            res.message = "儲存風險區域失敗"
        return res

    # ============================================================
    # 風險區域載入服務
    # ============================================================
    def risk_zone_load_srv(self, req, res):
        self.get_logger().info("載入風險區域")
        if self._load_risk_zone_list():
            self.get_logger().info(f"成功載入風險區域列表: {len(self.risk_zone_list.markers)} 個風險區域")
            self.risk_zone_list_pub.publish(self.risk_zone_list)
            res.success = True
            res.message = "成功載入風險區域"
        else:
            res.success = False
            res.message = "載入風險區域失敗"
        return res

    # def get_record_info(self, res):
    def get_record_zone_info_srv(self, req, res):
        self.get_logger().info(f"獲取普通區域列表，共 {len(self.record_zone_list.markers)} 個區域")
        # 將 MarkerArray 轉為字符串或僅列印IDs等概要資訊，避免傳入非str對象導致log報錯
        zone_ids = [marker.id for marker in self.record_zone_list.markers]
        self.get_logger().info(f"record_zone_list marker ids: {zone_ids}")
        res.success = True
        res.message = "返回test"
        # res.zone_list = self.record_zone_list
        return res

    # ============================================================
    # 獲取普通區域列表服務
    # ============================================================
    def get_record_zone_list_srv(self, req, res):
        """獲取當前記錄的普通區域列表"""
        try:
            self.get_logger().info(f"獲取普通區域列表，共 {len(self.record_zone_list.markers)} 個區域")
            res.success = True
            res.message = f"成功獲取普通區域列表，共 {len(self.record_zone_list.markers)} 個區域"
            res.zone_list = self.record_zone_list

            return res
        except Exception as e:
            self.get_logger().error(f"獲取普通區域列表失敗: {e}")
            res.success = False
            res.message = f"獲取普通區域列表失敗: {str(e)}"
            res.zone_list = MarkerArray()  # 返回空列表
            return res

    # ============================================================
    # 獲取風險區域列表服務
    # ============================================================
    def get_risk_zone_list_srv(self, req, res):
        """獲取當前記錄的風險區域列表"""
        try:
            self.get_logger().info(f"獲取風險區域列表，共 {len(self.risk_zone_list.markers)} 個風險區域")
            res.success = True
            res.message = f"成功獲取風險區域列表，共 {len(self.risk_zone_list.markers)} 個風險區域"
            res.zone_list = self.risk_zone_list
            return res
        except Exception as e:
            self.get_logger().error(f"獲取風險區域列表失敗: {e}")
            res.success = False
            res.message = f"獲取風險區域列表失敗: {str(e)}"
            res.zone_list = MarkerArray()  # 返回空列表
            return res

    # ============================================================
    # 儲存風險區域列表
    # ============================================================
    def _save_risk_zone_list(self):
        """儲存風險區域列表"""
        try:
            # 將MarkerArray轉換為可序列化的格式
            risk_zones_data = []
            for marker in self.risk_zone_list.markers:
                marker_data = {
                    'id': marker.id,
                    'ns': marker.ns,
                    'points': [[p.x, p.y, p.z] for p in marker.points]
                }
                risk_zones_data.append(marker_data)
            
            with open(self.get_parameter('save_dir').value + '/risk_zone_list.json', 'w') as f:
                json.dump(risk_zones_data, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f"儲存風險區域列表失敗: {e}")
            return False

    # ============================================================
    # 載入風險區域列表
    # ============================================================
    def _load_risk_zone_list(self):
        """載入風險區域列表"""
        try:
            with open(self.get_parameter('save_dir').value + '/risk_zone_list.json', 'r') as f:
                risk_zones_data = json.load(f)
            
            # 重建MarkerArray
            self.risk_zone_list = MarkerArray()
            for marker_data in risk_zones_data:
                marker = Marker()
                marker.header.frame_id = self.get_parameter('frame_id').value
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['ns']
                marker.id = marker_data['id']
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.03
                marker.color.a = 0.8
                marker.color.r = 1.0  # 紅色
                marker.color.g = 0.0
                marker.color.b = 0.0
                
                # 重建點列表
                marker.points = []
                for point_data in marker_data['points']:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)
                
                self.risk_zone_list.markers.append(marker)
            
            return True
        except Exception as e:
            self.get_logger().error(f"載入風險區域列表失敗: {e}")
            return False

    # ============================================================
    # 儲存普通區域列表
    # ============================================================
    def _save_zone_list(self):
        """改進普通區域儲存功能"""
        try:
            # 將MarkerArray轉換為可序列化的格式
            zones_data = []
            for marker in self.record_zone_list.markers:
                marker_data = {
                    'id': marker.id,
                    'ns': marker.ns,
                    'points': [[p.x, p.y, p.z] for p in marker.points]
                }
                zones_data.append(marker_data)
            
            with open(self.get_parameter('save_dir').value + '/zone_list.json', 'w') as f:
                json.dump(zones_data, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f"儲存區域列表失敗: {e}")
            return False

    # ============================================================
    # 載入普通區域列表
    # ============================================================
    def _load_zone_list(self):
        """改進普通區域載入功能"""
        try:
            with open(self.get_parameter('save_dir').value + '/zone_list.json', 'r') as f:
                zones_data = json.load(f)
            
            # 重建MarkerArray
            self.record_zone_list = MarkerArray()
            for marker_data in zones_data:
                marker = Marker()
                marker.header.frame_id = self.get_parameter('frame_id').value
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = marker_data['ns']
                marker.id = marker_data['id']
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.a = 0.5
                marker.color.r = 0.6  # 藍色
                marker.color.g = 0.0
                marker.color.b = 1.0
                
                # 重建點列表
                marker.points = []
                for point_data in marker_data['points']:
                    point = Point()
                    point.x = point_data[0]
                    point.y = point_data[1]
                    point.z = point_data[2]
                    marker.points.append(point)
                
                self.record_zone_list.markers.append(marker)
            
            return True
        except Exception as e:
            self.get_logger().error(f"載入區域列表失敗: {e}")
            return False

    # ============================================================
    # 啟動時自動載入區域資料
    # ============================================================
    def auto_load_zones_on_startup(self):
        """啟動時自動載入並發布區域資料"""
        # 只執行一次
        self.auto_load_zones_on_startup = lambda: None
        
        # 載入普通區域
        if self._load_zone_list():
            self.get_logger().info(f"啟動時自動載入區域列表: {len(self.record_zone_list.markers)} 個區域")
            self.zone_list_pub.publish(self.record_zone_list)
        
        # 載入風險區域
        if self._load_risk_zone_list():
            self.get_logger().info(f"啟動時自動載入風險區域列表: {len(self.risk_zone_list.markers)} 個風險區域")
            self.risk_zone_list_pub.publish(self.risk_zone_list)
    
def main():
    rclpy.init()
    node = PathRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()