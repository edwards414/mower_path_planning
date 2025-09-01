# path_recorder.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time as TimeMsg
import math, csv, os, time
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

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

        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.srv = self.create_service(Trigger, '/save_path', self.on_save)
        # 啟用marker發布器
        self.marker_pub = self.create_publisher(Marker, '/recorded_path_points', 1)
        # 新增多边形发布器
        self.polygon_pub = self.create_publisher(Marker, '/recorded_path_polygon', 1)

        self.path = Path()
        self.path.header.frame_id = self.get_parameter('frame_id').value
        self.last_pt = None
        self.last_t = self.get_clock().now()

        os.makedirs(self.get_parameter('save_dir').value, exist_ok=True)
        self.get_logger().info('path_recorder ready.')

        # 新增一個定時器，定時發布path，確保即使鍵盤遙控時也能看到path
        self.timer_period = 0.1  # 10Hz
        self.timer = self.create_timer(self.timer_period, self.publish_path_timer)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 改進初始化：設置為None，讓第一次獲取成功時自動初始化
        self.last_robot_pos = None
        self.initialized = False
        
        # 添加初始化定時器，等待TF可用
        self.init_timer = self.create_timer(0.5, self.try_initialize)

    def try_initialize(self):
        """嘗試初始化機器人位置"""
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
    
    def simplify_path(self, poses, distance_threshold):
        """
        使用Douglas-Peucker算法简化路径
        """
        if len(poses) < 3:
            return poses
            
        points = [(p.pose.position.x, p.pose.position.y) for p in poses]
        simplified_indices = self.douglas_peucker(points, distance_threshold)
        return [poses[i] for i in simplified_indices]

    def douglas_peucker(self, points, epsilon):
        """
        Douglas-Peucker算法实现
        """
        if len(points) < 3:
            return list(range(len(points)))
            
        # 找到距离起点终点连线最远的点
        start = points[0]
        end = points[-1]
        max_dist = 0
        max_index = 0
        
        for i in range(1, len(points) - 1):
            dist = self.point_to_line_distance(points[i], start, end)
            if dist > max_dist:
                max_dist = dist
                max_index = i
        
        # 如果最大距离大于阈值，递归处理
        if max_dist > epsilon:
            # 递归处理前半段和后半段
            left_indices = self.douglas_peucker(points[:max_index + 1], epsilon)
            right_indices = self.douglas_peucker(points[max_index:], epsilon)
            
            # 合并结果，注意避免重复
            result = left_indices + [max_index + i for i in right_indices[1:]]
            return result
        else:
            # 如果最大距离小于阈值，只保留起点和终点
            return [0, len(points) - 1]

    def point_to_line_distance(self, point, line_start, line_end):
        """
        计算点到直线的距离
        """
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end
        
        # 如果线段长度为0，返回点到点的距离
        line_length_sq = (x2 - x1) ** 2 + (y2 - y1) ** 2
        if line_length_sq == 0:
            return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)
        
        # 计算点到直线的距离
        numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        return numerator / math.sqrt(line_length_sq)

    def create_polygon_from_path(self, poses):
        """
        根据路径创建多边形
        """
        if len(poses) < 3:
            return None
            
        # 简化路径以减少多边形顶点数量
        simplify_dist = self.get_parameter('polygon_simplify_dist').value
        simplified_poses = self.simplify_path(poses, simplify_dist)
        
        # 创建多边形marker
        marker = Marker()
        marker.header.frame_id = self.path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "path_polygon"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # 线条宽度
        marker.color.a = 0.8   # 透明度
        marker.color.r = 0.0
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

    def publish_path_timer(self):
        # 如果還沒初始化，先嘗試初始化
        if not self.initialized:
            return
            
        robot_pos = self.get_robot_pos()
        now = self.get_clock().now()
        
        # 如果當前位置獲取失敗，跳過這次記錄
        if robot_pos is None:
            return
        print(robot_pos.pose.position.x, robot_pos.pose.position.y)
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
                polygon_marker = self.create_polygon_from_path(self.path.poses)
                if polygon_marker is not None:
                    self.polygon_pub.publish(polygon_marker)

    # 在rviz上標注行走紀錄的點位
    def publish_points_marker(self, poses):
        marker = Marker()
        marker.header.frame_id = self.path.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "recorded_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # 點的大小
        marker.scale.y = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.points = []
        for p in poses:
            pt = p.pose.position
            marker.points.append(pt)
        self.marker_pub.publish(marker)

    def on_save(self, req, res):
        if not self.path.poses:
            res.success = False
            res.message = 'No poses recorded.'
            return res
        ts = time.strftime('%Y%m%d_%H%M%S')
        base = os.path.join(self.get_parameter('save_dir').value, f'run_{ts}')
        csv_path = base + '.csv'
        yaml_path = base + '.yaml'

        # CSV: x,y,theta
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x','y','qx','qy','qz','qw'])
            for p in self.path.poses:
                q = p.pose.orientation
                w.writerow([p.pose.position.x, p.pose.position.y, q.x, q.y, q.z, q.w])

        # YAML: 直接丟 Path 也可，這裡存簡單 meta
        with open(yaml_path, 'w') as f:
            f.write(f'frame_id: {self.path.header.frame_id}\n')
            f.write(f'poses: {len(self.path.poses)}\n')
            f.write(f'csv_path: {csv_path}\n')

        res.success = True
        res.message = f'Saved to {csv_path}'
        self.get_logger().info(res.message)
        return res

def main():
    rclpy.init()
    n = PathRecorder()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
