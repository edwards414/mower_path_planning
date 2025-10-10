
import rclpy
from rclpy.node import Node
from nav_msgs.msg import  Path
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import Buffer, TransformListener

import math, os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
# from boustrophedon_coverage.path_record_utils import simplify_path
from boustrophedon_coverage.path_record_utils import *
class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_dist', 0.05)   # 最小移動距離(公尺)才記錄
        self.declare_parameter('min_dt', 0.10)     # 最小時間間隔(秒)才記錄
        self.declare_parameter('frame_id', 'map') # Path frame
        self.declare_parameter('save_dir', 'recordings')
        # 新增多边形相关参数
        self.declare_parameter('polygon_simplify_dist', 0.2)  # 多边形简化距离
        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)  

        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)

        # 設置與 boustrophedon_coverage 相同的 QoS
        polygon_qos = QoSProfile(depth=1)
        polygon_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        polygon_qos.reliability = QoSReliabilityPolicy.RELIABLE
        
        # 使用 QoS 創建多邊形發布者
        # 新增區域點標記發布者
        self.zone_marker_pub = self.create_publisher(Marker, '/zone_markers', polygon_qos)
        self.zone_list_pub = self.create_publisher(MarkerArray,'/zone_list', polygon_qos)

        self.get_logger().info('path_recorder ready.')

        # 新增一個定時器，定時發布path，確保即使鍵盤遙控時也能看到path
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.publish_path_timer)

        # 現有服務
        # self.create_service(SetBool, '/record_path_status', self.record_path_status_srv)
        # 新增區域記錄服務
        self.create_service(Trigger, '/record_zone_start', self.record_zone_start_srv)
        self.create_service(Trigger, '/record_zone_end', self.record_zone_end_srv)

        self.record_zone_status = False
        self.record_zone_id = 0
        self.record_zone_name = "zone_" + str(self.record_zone_id)
        self.record_zone_marker = Marker()#記錄當前的zone point
        self.record_zone_list = MarkerArray() #全域zone list
        
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
        
    # ============================================================
    # 嘗試初始化機器人位置
    # ============================================================
    def try_initialize(self):
        if not self.initialized:
            robot_pos = self.get_robot_pos()
            if robot_pos is not None:
                self.last_robot_pos = robot_pos
                self.initialized = True
                self.get_logger().info(f"機器人位置初始化成功: x={robot_pos.pose.position.x:.3f}, y={robot_pos.pose.position.y:.3f}")
                # 取消初始化定時器
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
    # 根据路径创建多边形
    # ============================================================
    def create_polygon_from_path(self, poses):
        if len(poses) < 3:
            return None
        simplify_dist = self.get_parameter('polygon_simplify_dist').value

        # 简化路径以减少多边形顶点数量
        simplified_poses = simplify_path(poses, simplify_dist)
        
        # 创建多边形marker
        marker = Marker()
        marker.header.frame_id = self.path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "zones"
        marker.id = self.record_zone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 线条宽度
        marker.color.a = 0.5   # 透明度
        marker.color.r = 0.6
        marker.color.g = 0.0
        marker.color.b = 1.0   # 蓝色
        
        # 添加所有点
        marker.points = []
        for pose in simplified_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            marker.points.append(point)
            
        # 如果路径是封闭的（起点和终点接近），则添加起点作为最后一个点形成封闭多边形
        if len(simplified_poses) > 2:
            first_pt = simplified_poses[0].pose.position
            last_pt = simplified_poses[-1].pose.position
            dist = math.sqrt((first_pt.x - last_pt.x)**2 + (first_pt.y - last_pt.y)**2)
            
            # 如果起点和终点距离较近，认为是封闭路径
            if dist < 1.0:  # 1米内认为是封闭的
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
        if self.record_zone_status == False:
            return
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
            
            # 发布多边形
            if len(self.path.poses) >= 3:
                self.record_zone_marker = self.create_polygon_from_path(self.path.poses)
                if self.record_zone_marker is not None:
                    self.zone_marker_pub.publish(self.record_zone_marker)

    # ============================================================
    # 記錄區域起始點 新增zone 區域
    # ============================================================
    def record_zone_start_srv(self, req, res):
        self.get_logger().info("記錄區域起始點")
        self.record_zone_status = True
        self.record_zone_id += 1
        self.record_zone_name = "zone_" + str(self.record_zone_id)

        # 清空路径，开始记录新的区域
        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value

        # 重新初始化机器人位置，确保第二个zone可以正常开始记录
        robot_pos = self.get_robot_pos()
        if robot_pos is not None:
            self.last_robot_pos = robot_pos
            self.get_logger().info(f"重新初始化機器人位置: x={robot_pos.pose.position.x:.3f}, y={robot_pos.pose.position.y:.3f}")

        self.record_zone_marker = Marker()
        self.zone_marker_pub.publish(self.record_zone_marker)#刷新zone marker

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

        self.zone_list_pub.publish(self.record_zone_list) #發佈全域zone list

        res.success = True
        res.message = f"成功記錄區域結束點 #{len(self.record_zone_list.markers)}"
        return res

def main():
    rclpy.init()
    node = PathRecorder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()