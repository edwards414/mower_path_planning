class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_coverage')

        # 參數
        self.declare_parameter('strip_width_m', 0.30)          # 割草機有效割幅
        self.declare_parameter('waypoint_spacing_m', 0.15)     # 路徑點間距
        self.declare_parameter('free_threshold', 25)           # 佔據格 <= 此值視為可行
        self.declare_parameter('unknown_as_obstacle', True)    # 未知(-1)是否當作障礙
        self.declare_parameter('inflate_radius_m', 0.15)       # 安全膨脹半徑(機身+裕度)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos
        )

        self.map_split_line = self.create_publisher(Marker, '/coverage_split_lines', 1)
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)

    def on_map(self, map_msg: OccupancyGrid):
        info = map_msg.info
        H, W = info.height, info.width
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        occ = np.asarray(map_msg.data, dtype=np.int16).reshape(H, W)
        free_th = int(self.get_parameter('free_threshold').value)
        unknown_as_obstacle = bool(self.get_parameter('unknown_as_obstacle').value)

        # 建立可行遮罩
        free_mask = (occ >= 0) & (occ <= free_th)
        if unknown_as_obstacle:
            free_mask &= (occ >= 0)

        # 障礙膨脹
        inflate_r_m = float(self.get_parameter('inflate_radius_m').value)
        r_cells = max(0, int(math.ceil(inflate_r_m / res)))
        if r_cells > 0:
            occ_mask = (~free_mask).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            occ_mask = cv2.dilate(occ_mask, k)
            free_mask = (occ_mask == 0)

        # 條帶設定：以 X 方向切直條(沿 Y 掃描)
        strip_w_m = float(self.get_parameter('strip_width_m').value)
        strip_cols = max(1, int(round(strip_w_m / res)))
        midcols = list(range(strip_cols // 2, W, strip_cols))

        # 視覺化條帶分割線
        cols_for_viz = max(1, int(math.ceil(W / strip_cols)))
        self.publish_split_lines(map_msg, rows=1, cols=cols_for_viz, line_width=0.03)

        # 產生往返路徑
        spacing = float(self.get_parameter('waypoint_spacing_m').value)
        points = []
        reverse = False

        for mc in midcols:
            # 沿條帶中心列，找連續可行段
            segments = []
            start = None
            for i in range(H):
                ok = bool(free_mask[i, mc])
                is_last = (i == H - 1)
                if ok and start is None:
                    start = i
                if (not ok or is_last) and start is not None:
                    end = i if (not ok) else i
                    segments.append((start, end))
                    start = None

            # 交替方向，形成牛耕(往返)
            segs = segments[::-1] if reverse else segments
            for (s, e) in segs:
                y0 = oy + (s + 0.5) * res
                y1 = oy + (e + 0.5) * res
                x  = ox + (mc + 0.5) * res
                # densify
                if y1 >= y0:
                    ys = list(np.arange(y0, y1, max(res, spacing))) + [y1]
                else:
                    ys = list(np.arange(y0, y1, -max(res, spacing))) + [y1]
                ys = ys[::-1] if reverse else ys
                points.extend([(x, y) for y in ys])

            reverse = not reverse

        # 發布 Path
        path = Path()
        path.header = map_msg.header
        path.header.frame_id = map_msg.header.frame_id or 'map'
        for (x, y) in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_split_lines(self, map_msg: OccupancyGrid, rows: int, cols: int, line_width: float = 0.03):
        info = map_msg.info
        w, h = info.width, info.height
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y

        marker = Marker()
        marker.header.frame_id = map_msg.header.frame_id or 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'split'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = line_width
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        def add_line(x0, y0, x1, y1):
            p0 = Point(x=float(x0), y=float(y0), z=0.0)
            p1 = Point(x=float(x1), y=float(y1), z=0.0)
            marker.points.append(p0)
            marker.points.append(p1)

        for k in range(1, cols):
            j = (w * k) // cols
            x = ox + j * res
            add_line(x, oy, x, oy + h * res)

        for k in range(1, rows):
            i = (h * k) // rows
            y = oy + i * res
            add_line(ox, y, ox + w * res, y)

        self.map_split_line.publish(marker)
        self.get_logger().info('split lines published')