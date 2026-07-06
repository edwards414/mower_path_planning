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
import copy
import math
import threading

from mower_interface.msg import ZoneMap
from mower_interface.srv import GetZoneList, ImportImageMask, ZoneMapList

import cv2

from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
from rcl_interfaces.msg import SetParametersResult

from mower_interface.srv import ChennalPathList

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_srvs.srv import Trigger

from mower_mission.image_mask_import import (
    decode_u8_mask,
    rasterize_image_masks,
    yaw_from_quaternion,
)


class MapManage(Node):

    def __init__(self):
        super().__init__('map_manage')
        self.get_logger().info('map_manage init')
        # Aligned with Nav2 robot_radius (0.5 m) + coverage_clearance (0.05 m)
        self.declare_parameter('inflate_radius_m', 0.55)
        self.add_on_set_parameters_callback(self.on_parameters_changed)
        self.set_parameters([
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_service(Trigger, '/create_risk_map', self.create_risk_map_srv)
        self.create_service(Trigger, '/create_free_space', self.create_free_space_srv)
        self.create_service(
            ImportImageMask, '/import_image_mask', self.import_image_mask_srv
        )

        self.create_service(Trigger, '/create_chennal_map', self.create_chennal_map_srv)
        self.create_service(
            ZoneMapList, '/get_zone_map_list_srv', self.get_zone_map_list_srv
        )
        self.create_service(
            Trigger, '/restore_free_space_coverage',
            self.restore_free_space_srv,
        )

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

        self.free_space_pub = self.create_publisher(OccupancyGrid, '/free_space', qos)
        self.free_space_inflated_pub = self.create_publisher(
            OccupancyGrid, '/free_space_inflated', qos
        )
        self.risk_map_pub = self.create_publisher(OccupancyGrid, '/risk_map', qos)
        self.risk_map_inflated_pub = self.create_publisher(
            OccupancyGrid, '/risk_map_inflated', qos
        )

        self.latest_map = None
        self.zone_list = []
        self.zone_map_list = []
        self.base_map = None
        # Robot-collected freespace (from /create_free_space), persisted
        # separately so /import_image_mask never overwrites it — used to clip
        # the imported image mask to the real drivable area.
        self.collected_free_space = None
        self.risk_map = None
        self.chennal_map = None
        # Image-mission swap-out: /import_image_mask backs up the freespace zone
        # maps + risk here so the app can restore them (discarding the image)
        # when switching back to zigzag/spiral via /restore_free_space_coverage.
        self._free_zone_backup = None
        self._free_risk_backup = None

        self.get_chennal_path_list_client = self.create_client(
            ChennalPathList, '/get_chennal_path_list',
            callback_group=self.service_callback_group
        )

        self.chennal_map_pub = self.create_publisher(OccupancyGrid, '/chennal_map', qos)
        self.chennal_map_inflated_pub = self.create_publisher(
            OccupancyGrid, '/chennal_map_inflated', qos
        )

        self.declare_parameter('chennal_width_m', 0.6)

        self.nav_base_map_pub = self.create_publisher(OccupancyGrid, '/map_grid', qos)
        self.resolution = 0.1
        self.width = 400
        self.height = 400

        self.map_msg = self.build_demo_map()

        self.timer = self.create_timer(1.0, self.timer_cb)
        self.get_logger().info(f'Publishing OccupancyGrid on /map_grid')

    def on_parameters_changed(self, params):
        """Validate runtime parameters and refresh derived inflated maps."""
        next_inflate_radius_m = None
        for param in params:
            if param.name != 'inflate_radius_m':
                continue

            try:
                next_inflate_radius_m = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason='inflate_radius_m must be a number'
                )

            if next_inflate_radius_m < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason='inflate_radius_m must be >= 0'
                )

        if next_inflate_radius_m is not None:
            self.refresh_inflated_maps(next_inflate_radius_m)

        return SetParametersResult(successful=True)

    def get_inflate_radius_m(self, override=None):
        """Return the active inflate radius, or a pending parameter value."""
        if override is not None:
            return float(override)
        return float(self.get_parameter('inflate_radius_m').value)

    def refresh_inflated_maps(self, inflate_radius_m=None):
        """Recalculate inflated maps after inflate_radius_m changes."""
        refreshed_topics = []

        if self.base_map is not None:
            for zone_map in self.zone_map_list:
                zone_map.mask_map_inflated = self._create_free_space_inflated(
                    zone_map.mask_map,
                    inflate_radius_m,
                )

            # Keep /free_space_inflated = collected freespace (the app's
            # alignment background) when it exists; only fall back to base_map
            # (which after an import holds the clipped image map) otherwise.
            free_space_source = (
                self.collected_free_space
                if self.collected_free_space is not None
                else self.base_map
            )
            free_space_inflated = self._create_free_space_inflated(
                free_space_source,
                inflate_radius_m,
            )
            if free_space_inflated is not None:
                self.free_space_inflated_pub.publish(free_space_inflated)
                refreshed_topics.append('/free_space_inflated')

        if self.risk_map is not None:
            risk_map_inflated = self._create_risk_map_inflated(
                self.risk_map,
                inflate_radius_m,
            )
            if risk_map_inflated is not None:
                self.risk_map_inflated_pub.publish(risk_map_inflated)
                refreshed_topics.append('/risk_map_inflated')

        if self.chennal_map is not None:
            chennal_map_inflated = self._create_chennal_map_inflated(
                self.chennal_map,
                inflate_radius_m,
            )
            if chennal_map_inflated is not None:
                self.chennal_map_inflated_pub.publish(chennal_map_inflated)
                refreshed_topics.append('/chennal_map_inflated')

        radius = self.get_inflate_radius_m(inflate_radius_m)
        if refreshed_topics:
            self.get_logger().info(
                f'inflate_radius_m updated to {radius:.2f}; refreshed '
                f'{", ".join(refreshed_topics)}'
            )
        else:
            self.get_logger().info(
                f'inflate_radius_m updated to {radius:.2f}; no generated maps '
                'to refresh yet'
            )

    def build_demo_map(self) -> OccupancyGrid:
        msg = OccupancyGrid()

        msg.header.frame_id = 'map'
        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        origin = Pose()
        origin.position.x = -20.0
        origin.position.y = -20.0
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        grid = np.zeros((self.height, self.width), dtype=np.int8)

        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100

        msg.data = grid.flatten().tolist()
        return msg

    def timer_cb(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.nav_base_map_pub.publish(self.map_msg)

    def import_image_mask_srv(self, req, res):
        """Import an app-generated black/white mask as the active zone map."""
        try:
            free_map, risk_map, zone_map, area_m2 = self._create_image_mask_maps(
                req
            )
        except ValueError as exc:
            res.success = False
            res.message = str(exc)
            res.zone_id = int(req.zone_id)
            res.area_m2 = 0.0
            return res
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'import image mask failed: {exc!r}')
            res.success = False
            res.message = '圖片 mask 匯入失敗'
            res.zone_id = int(req.zone_id)
            res.area_m2 = 0.0
            return res

        free_space_inflated = self._create_free_space_inflated(free_map)
        risk_map_inflated = self._create_risk_map_inflated(risk_map)
        # Custom (image) coverage uses NO inflation so the swept path hugs the
        # uploaded shape: the area is already clipped to freespace, and the
        # outer-contour ring (coverage_node boundary_ring) traces the edge.
        zone_map.mask_map_inflated = copy.deepcopy(zone_map.mask_map)

        # The image is only a RANGE LIMITER. zone_map.mask_map already holds
        # image_free ∩ collected_freespace (see _create_image_mask_maps), so the
        # coverage PATH is the intersection no matter which freespace we keep as
        # base_map / display.
        # Back up the freespace zone maps + risk (only once, so a re-import does
        # not clobber it) so /restore_free_space_coverage can bring them back.
        if self._free_zone_backup is None:
            self._free_zone_backup = self.zone_map_list
            self._free_risk_backup = self.risk_map
        self.zone_map_list = [zone_map]
        # Risk always comes from the image so the image's risk mask applies to
        # this mission. coverage_node resamples /risk_map_inflated onto the zone
        # grid, and the zone grid IS the image grid, so image-grid risk is exact
        # even when base_map keeps the (different) collected grid below.
        self.risk_map = risk_map

        if self.collected_free_space is None:
            # No collected freespace -> image-only behavior (UNCHANGED): the
            # image becomes the base / free-space / display map.
            self.base_map = free_map
            self.map_msg = free_map
            self.free_space_pub.publish(free_map)
            self.free_space_inflated_pub.publish(free_space_inflated)
            self.nav_base_map_pub.publish(free_map)
        else:
            # Collected freespace exists -> KEEP it. Do NOT republish
            # /free_space, /free_space_inflated or /map_grid with the image: the
            # latched collected layers stay (green display preserved), and
            # base_map/map_msg keep the collected grid that create_risk_map /
            # create_chennal_map / refresh_inflated_maps / the /map_grid timer
            # rely on. The coverage path is still image ∩ collected via the
            # zone map above. (Re-asserts collected as a safety net.)
            self.base_map = self.collected_free_space
            self.map_msg = self.collected_free_space

        # Risk is published from the IMAGE in both branches.
        self.risk_map_pub.publish(risk_map)
        self.risk_map_inflated_pub.publish(risk_map_inflated)

        res.success = True
        res.message = '圖片 mask 匯入成功'
        res.zone_id = zone_map.zone_id
        res.area_m2 = area_m2
        self.get_logger().info(
            f'imported image mask zone={zone_map.zone_id}, '
            f'area={area_m2:.2f} m^2, size={free_map.info.width}x'
            f'{free_map.info.height}'
        )
        return res

    def restore_free_space_srv(self, req, res):
        """Discard the imported image coverage and restore the freespace zone
        maps + risk active before /import_image_mask. Called by the app when it
        switches back to zigzag/spiral so coverage uses the full freespace."""
        if self._free_zone_backup is None:
            res.success = True
            res.message = '目前已是自由空間覆蓋'
            return res
        self.zone_map_list = self._free_zone_backup
        self.risk_map = self._free_risk_backup
        self._free_zone_backup = None
        self._free_risk_backup = None
        # Republish the freespace risk so coverage_node resamples it (the image
        # risk was published on /risk_map_inflated during the import).
        if self.risk_map is not None:
            self.risk_map_pub.publish(self.risk_map)
            risk_inflated = self._create_risk_map_inflated(self.risk_map)
            if risk_inflated is not None:
                self.risk_map_inflated_pub.publish(risk_inflated)
        res.success = True
        res.message = '已還原為完整自由空間覆蓋'
        self.get_logger().info('restored freespace coverage (image discarded)')
        return res

    def _create_image_mask_maps(self, req):
        if req.mask_encoding != 'base64_u8_row_major':
            raise ValueError('mask_encoding must be base64_u8_row_major')

        width = int(req.width)
        height = int(req.height)
        resolution = float(req.resolution_m)
        if width <= 0 or height <= 0:
            raise ValueError('圖片 mask 尺寸無效')
        if resolution <= 0.0:
            raise ValueError('resolution_m must be > 0')

        free_mask = self._decode_u8_mask(
            req.free_mask_data, width, height, field_name='free_mask_data'
        )
        risk_mask = self._decode_u8_mask(
            req.risk_mask_data,
            width,
            height,
            field_name='risk_mask_data',
            optional=True,
        )

        if np.count_nonzero(free_mask == 255) == 0:
            raise ValueError('free_mask_data 沒有可割草白色區域')

        free_grid, risk_grid, origin_x, origin_y = self._rasterize_image_masks(
            free_mask=free_mask,
            risk_mask=risk_mask,
            resolution=resolution,
            robot_x=float(req.robot_pose_map.position.x),
            robot_y=float(req.robot_pose_map.position.y),
            robot_yaw=self._yaw_from_quaternion(req.robot_pose_map.orientation),
            start_x=float(req.start_x_m),
            start_y=float(req.start_y_m),
            image_heading=float(req.image_heading_rad),
        )

        # Clip the imported mask to the robot-collected freespace so the mowable
        # region is image_free ∩ collected_freespace (risk is removed later by
        # coverage_node). No-op when no freespace was ever collected.
        had_collected = self.collected_free_space is not None
        free_grid = self._clip_free_grid_to_collected(
            free_grid, origin_x, origin_y, resolution
        )
        if had_collected and np.count_nonzero(free_grid == 0) == 0:
            raise ValueError('圖片與採集的 freespace 沒有重疊，無法產生路徑')

        header = copy.deepcopy(req.robot_pose_header)
        header.frame_id = header.frame_id or 'map'
        header.stamp = self.get_clock().now().to_msg()

        free_map = self._occupancy_grid_from_array(
            free_grid,
            header=header,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )
        risk_map = self._occupancy_grid_from_array(
            risk_grid,
            header=copy.deepcopy(header),
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

        zone_map = ZoneMap()
        zone_map.header = copy.deepcopy(header)
        zone_map.zone_id = int(req.zone_id) if int(req.zone_id) > 0 else 9001
        zone_map.mask_map = copy.deepcopy(free_map)
        area_m2 = float(np.count_nonzero(free_grid == 0)) * resolution * resolution
        return free_map, risk_map, zone_map, area_m2

    def _clip_free_grid_to_collected(self, free_grid, origin_x, origin_y, resolution):
        """Intersect an image free_grid with the robot-collected freespace.

        Both use the OccupancyGrid convention 0=free, 100=occupied. A cell stays
        free (0) only where it is free in BOTH grids; everything else becomes 100.
        The grids may differ in origin/resolution, so the collected grid is
        nearest-cell resampled into the image grid (same pattern as
        coverage_node._resample_risk_map_to_zone): each image cell centre ->
        world metres -> floor into the collected grid. Out-of-bounds or unknown
        (value != 0) collected cells are treated as occupied / not-free.

        Returns ``free_grid`` unchanged when no freespace has been collected.
        """
        collected = self.collected_free_space
        if collected is None:
            return free_grid

        img_h, img_w = free_grid.shape
        col_h = int(collected.info.height)
        col_w = int(collected.info.width)
        col_res = collected.info.resolution
        col_ox = collected.info.origin.position.x
        col_oy = collected.info.origin.position.y
        collected_data = np.asarray(
            collected.data, dtype=np.int16
        ).reshape(col_h, col_w)

        # image cell centre -> world metres -> collected grid index
        yy, xx = np.indices((img_h, img_w))
        world_x = origin_x + (xx + 0.5) * resolution
        world_y = origin_y + (yy + 0.5) * resolution
        col_cols = np.floor((world_x - col_ox) / col_res).astype(np.int64)
        col_rows = np.floor((world_y - col_oy) / col_res).astype(np.int64)

        inside = (
            (col_rows >= 0)
            & (col_rows < col_h)
            & (col_cols >= 0)
            & (col_cols < col_w)
        )
        collected_free = np.zeros((img_h, img_w), dtype=bool)
        collected_free[inside] = (
            collected_data[col_rows[inside], col_cols[inside]] == 0
        )

        return np.where(
            (free_grid == 0) & collected_free, 0, 100
        ).astype(np.int8)

    @staticmethod
    def _decode_u8_mask(
        encoded,
        width,
        height,
        *,
        field_name,
        optional=False,
    ):
        return decode_u8_mask(
            encoded,
            width,
            height,
            field_name=field_name,
            optional=optional,
        )

    @staticmethod
    def _yaw_from_quaternion(q):
        return yaw_from_quaternion(q)

    @staticmethod
    def _rasterize_image_masks(
        *,
        free_mask,
        risk_mask,
        resolution,
        robot_x,
        robot_y,
        robot_yaw,
        start_x,
        start_y,
        image_heading,
    ):
        return rasterize_image_masks(
            free_mask=free_mask,
            risk_mask=risk_mask,
            resolution=resolution,
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            start_x=start_x,
            start_y=start_y,
            image_heading=image_heading,
        )

    @staticmethod
    def _occupancy_grid_from_array(grid, *, header, resolution, origin_x, origin_y):
        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = msg.header.frame_id or 'map'
        msg.info = MapMetaData()
        msg.info.resolution = resolution
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])

        origin = Pose()
        origin.position.x = float(origin_x)
        origin.position.y = float(origin_y)
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin
        msg.data = grid.astype(np.int8).flatten().tolist()
        return msg

    def _wait_for_future(self, future, timeout_sec):
        """Block until an rclpy Future completes; return True, or False on
        timeout. The response is serviced by another executor thread
        (MultiThreadedExecutor) because the client lives in a ReentrantCallback
        group distinct from this service callback — so this wait cannot deadlock.
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout=timeout_sec)

    def create_risk_map_srv(self, req, res):
        """創建風險地圖服務（同步、回報真實成敗）."""
        self.get_logger().info('(service)create_risk_map_srv call')
        try:
            res.success, res.message = self._create_risk_map_sync()
        except Exception as e:
            self.get_logger().error(f'創建風險地圖時發生錯誤: {e}')
            res.success = False
            res.message = f'創建風險地圖時發生錯誤: {e}'
        return res

    def _create_risk_map_sync(self, timeout_sec=30.0):
        """Build + publish the risk map, returning (success, message).

        Blocks on the nested /get_risk_zone_list call; safe against deadlock
        because this service callback and the client response run in different
        callback groups on a MultiThreadedExecutor.
        """
        if not self.get_risk_zone_list_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('風險區域列表服務不可用')
            return False, '風險區域列表服務不可用'

        future = self.get_risk_zone_list_client.call_async(GetZoneList.Request())
        if not self._wait_for_future(future, timeout_sec):
            self.get_logger().error('獲取風險區域列表逾時')
            return False, '獲取風險區域列表逾時'

        risk_zone_response = future.result()
        if not risk_zone_response.success:
            self.get_logger().error(
                f'獲取風險區域列表失敗: {risk_zone_response.message}'
            )
            return False, f'獲取風險區域列表失敗: {risk_zone_response.message}'

        risk_map = self._generate_risk_map(risk_zone_response.zone_list)
        if risk_map is None:
            return (
                False,
                '生成風險地圖失敗（可能尚未建立自由空間，請先呼叫 /create_free_space）',
            )

        self.risk_map = risk_map
        risk_map_inflated = self._create_risk_map_inflated(risk_map)
        self.risk_map_pub.publish(risk_map)
        self.risk_map_inflated_pub.publish(risk_map_inflated)
        n = len(risk_zone_response.zone_list.markers)
        self.get_logger().info(f'成功創建風險地圖，包含 {n} 個風險區域')
        return True, f'成功創建風險地圖，包含 {n} 個風險區域'

    def _generate_risk_map(self, risk_zones):
        """根據基礎地圖和風險區域生成風險地圖."""
        try:
            if self.base_map is None:
                self.get_logger().error(
                    '風險地圖需要先建立自由空間（請先呼叫 /create_free_space）'
                )
                return None
            risk_map = OccupancyGrid()
            risk_map.header = self.base_map.header
            risk_map.header.frame_id = 'map'
            risk_map.info = self.base_map.info

            W = self.base_map.info.width
            H = self.base_map.info.height
            resolution = self.base_map.info.resolution
            ox = self.base_map.info.origin.position.x
            oy = self.base_map.info.origin.position.y

            risk_map_data = np.zeros((H, W), dtype=np.uint8)

            for polygon_points in risk_zones.markers:
                poly_px = []
                for pt in polygon_points.points:
                    x = int((pt.x - ox) / resolution)
                    y = int((pt.y - oy) / resolution)
                    x = max(0, min(x, W-1))
                    y = max(0, min(y, H-1))
                    poly_px.append([x, y])

                if len(poly_px) < 2:
                    continue

                poly_px_np = np.array(poly_px, dtype=np.int32)
                temp_mask = np.zeros((H, W), dtype=np.uint8)

                if len(poly_px) >= 3:
                    cv2.fillPoly(temp_mask, [poly_px_np], 1)

                closed = False
                if len(poly_px) >= 3:
                    first = np.array(poly_px[0])
                    last = np.array(poly_px[-1])
                    close_threshold_cells = max(
                        1,
                        int(math.ceil(1.0 / resolution))
                    )
                    closed = (
                        np.linalg.norm(first - last) <= close_threshold_cells
                    )

                cv2.polylines(
                    temp_mask,
                    [poly_px_np],
                    isClosed=closed,
                    color=1,
                    thickness=1,
                )
                risk_map_data = np.where(temp_mask == 1, 100, risk_map_data)

            risk_map.data = risk_map_data.flatten().tolist()
            return risk_map

        except Exception as e:
            self.get_logger().error(f'生成風險地圖時發生錯誤: {e}')
            return None

    def _create_risk_map_inflated(
        self,
        risk_map: OccupancyGrid,
        inflate_radius_m=None,
    ):
        """膨脹風險地圖."""
        if risk_map is None:
            return None

        inflate_r_m = self.get_inflate_radius_m(inflate_radius_m)
        resolution = risk_map.info.resolution
        H = risk_map.info.height
        W = risk_map.info.width
        risk_map_data = np.asarray(risk_map.data, dtype=np.int16).reshape(H, W)
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))
        if r_cells > 0:
            risk_binary = (risk_map_data == 100).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            risk_binary_inflated = cv2.dilate(risk_binary, k)
            inflated_data = np.where(risk_binary_inflated == 1, 100, 0).astype(np.int8)
            inflated_map = OccupancyGrid()
            inflated_map.header = risk_map.header
            inflated_map.info = risk_map.info
            inflated_map.data = inflated_data.flatten().tolist()
            return inflated_map
        else:
            return risk_map

    def create_free_space_srv(self, req, res):
        """創建自由空間服務（同步、回報真實成敗）."""
        self.get_logger().info('(service)create_free_space_srv call')
        try:
            res.success, res.message = self._create_free_space_sync()
        except Exception as e:
            self.get_logger().error(f'創建自由空間時發生錯誤: {e}')
            res.success = False
            res.message = f'創建自由空間時發生錯誤: {e}'
        return res

    def _create_free_space_sync(self, timeout_sec=30.0):
        """Build + publish the collected free-space map, returning
        (success, message). See _create_risk_map_sync for the deadlock note.
        """
        if not self.get_record_zone_list_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('記錄區域列表服務不可用')
            return False, '記錄區域列表服務不可用'

        future = self.get_record_zone_list_client.call_async(GetZoneList.Request())
        if not self._wait_for_future(future, timeout_sec):
            self.get_logger().error('獲取記錄區域列表逾時')
            return False, '獲取記錄區域列表逾時'

        zone_response = future.result()
        if not zone_response.success:
            self.get_logger().error(f'獲取記錄區域列表失敗: {zone_response.message}')
            return False, f'獲取記錄區域列表失敗: {zone_response.message}'

        overall_freespace_map = self._create_zone_maps_and_freespace(
            zone_response.zone_list
        )
        if overall_freespace_map is None:
            self.get_logger().error('生成自由空間失敗')
            return False, '生成自由空間失敗'

        overall_freespace_map_inflated = self._create_free_space_inflated(
            overall_freespace_map
        )
        self.base_map = overall_freespace_map
        # Persist the collected freespace so a later /import_image_mask clips to
        # it (this field is NOT overwritten by import, unlike base_map).
        self.collected_free_space = overall_freespace_map
        self.free_space_pub.publish(overall_freespace_map)
        self.free_space_inflated_pub.publish(overall_freespace_map_inflated)
        self.map_msg = overall_freespace_map
        n = len(self.zone_map_list)
        self.get_logger().info(f'成功創建自由空間，包含 {n} 個區域')
        return True, f'成功創建自由空間，包含 {n} 個區域'

    def _create_zone_maps_and_freespace(self, zone_list):
        self.zone_map_list = []
        # Invalidate any previously collected freespace; it is re-set on success
        # in _handle_zone_list_response. Prevents a failed/empty re-record from
        # leaving a stale clip mask behind.
        self.collected_free_space = None

        if not zone_list.markers:
            self.get_logger().warn('沒有區域數據')
            return None

        all_points = []
        for polygon_points in zone_list.markers:
            for pt in polygon_points.points:
                all_points.append((pt.x, pt.y))

        if not all_points:
            self.get_logger().warn('沒有有效的點數據')
            return None

        min_x = min(pt[0] for pt in all_points)
        max_x = max(pt[0] for pt in all_points)
        min_y = min(pt[1] for pt in all_points)
        max_y = max(pt[1] for pt in all_points)

        resolution = 0.05
        margin = 1.0

        map_width = max_x - min_x + 2 * margin
        map_height = max_y - min_y + 2 * margin
        W = int(map_width / resolution)
        H = int(map_height / resolution)

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
            poly_px = []
            for pt in polygon_points.points:
                x = int((pt.x - ox) / resolution)
                y = int((pt.y - oy) / resolution)
                poly_px.append([x, y])
            # A polygon needs at least 3 vertices. A marker with an empty or
            # degenerate point list makes cv2.fillPoly raise
            # (-215) p.checkVector(2, CV_32S) >= 0, which crashes the node and
            # leaves /free_space + /risk_map_inflated unpublished (coverage then
            # fails with "缺少 risk_map_inflated 地圖數據"). _generate_risk_map
            # above already guards this the same way.
            if len(poly_px) < 3:
                self.get_logger().warn(
                    f'zone {polygon_points.id}: 僅 {len(poly_px)} 個點，'
                    '無法構成多邊形，略過'
                )
                continue
            zone_mask = np.zeros((H, W), dtype=np.uint8)
            poly_px = np.array(poly_px, dtype=np.int32)
            cv2.fillPoly(zone_mask, [poly_px], 1)
            cv2.fillPoly(mask, [poly_px], 1)
            zone_occ_masked = np.where(zone_mask == 1, 0, 100)
            zone_map = ZoneMap()
            zone_map.zone_id = polygon_points.id
            zone_map.mask_map = copy.deepcopy(masked_map)
            zone_map.mask_map.data = zone_occ_masked.flatten().tolist()

            zone_map.mask_map_inflated = self._create_free_space_inflated(zone_map.mask_map)
            self.zone_map_list.append(zone_map)
            self.get_logger().info(
                f'成功創建zone map，包含 {len(self.zone_map_list)} 個區域'
            )

        occ_masked = np.where(mask == 1, 0, 100)
        masked_map.data = occ_masked.flatten().tolist()

        return masked_map

    def _create_free_space_inflated(
        self,
        free_space_map: OccupancyGrid,
        inflate_radius_m=None,
    ) -> OccupancyGrid:
        """向內膨脹自由空間地圖."""
        if free_space_map is None:
            return None

        inflate_r_m = self.get_inflate_radius_m(inflate_radius_m)
        resolution = free_space_map.info.resolution
        H = free_space_map.info.height
        W = free_space_map.info.width
        free_space_map_data = np.asarray(free_space_map.data, dtype=np.int16).reshape(H, W)
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))
        if r_cells > 0:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            free_mask = (free_space_map_data == 0).astype(np.uint8)
            eroded_free_mask = cv2.erode(free_mask, k)
            inflated_data = np.where(eroded_free_mask == 1, 0, 100).astype(np.int8)
            inflated_map = OccupancyGrid()
            inflated_map.header = free_space_map.header
            inflated_map.info = free_space_map.info
            inflated_map.data = inflated_data.flatten().tolist()
            return inflated_map
        else:
            return free_space_map

    def create_chennal_map_srv(self, req, res):
        """创建通道地图服务."""
        self.get_logger().info('(service)create_chennal_map_srv call')
        res.success = True
        res.message = '开始创建通道地图，请稍候...'
        self._start_create_chennal_map_async()
        return res

    def _start_create_chennal_map_async(self):
        """异步创建通道地图."""
        try:
            if not self.get_chennal_path_list_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('通道路径列表服务不可用')
                return

            chennal_req = ChennalPathList.Request()
            future = self.get_chennal_path_list_client.call_async(chennal_req)
            future.add_done_callback(self._handle_chennal_path_list_response)

        except Exception as e:
            self.get_logger().error(f'启动异步创建通道地图时发生错误: {e}')

    def _handle_chennal_path_list_response(self, future):
        """处理通道路径列表服务响应."""
        try:
            if not future.done():
                self.get_logger().error('获取通道路径列表超时')
                return

            chennal_response = future.result()

            if not chennal_response.success:
                self.get_logger().error(
                    f'获取通道路径列表失败: {chennal_response.message}'
                )
                return

            chennal_map = self._generate_chennal_map(
                chennal_response.chennal_path_array
            )
            if chennal_map is None:
                self.get_logger().error('生成通道地图失败')
                return

            self.chennal_map = chennal_map
            chennal_map_inflated = self._create_chennal_map_inflated(chennal_map)

            self.chennal_map_pub.publish(chennal_map)
            self.chennal_map_inflated_pub.publish(chennal_map_inflated)
            self.get_logger().info(
                f'成功创建通道地图，包含 '
                f'{len(chennal_response.chennal_path_array.markers)} 条通道'
            )

        except Exception as e:
            self.get_logger().error(f'处理通道路径列表响应时发生错误: {e}')

    def _generate_chennal_map(self, chennal_path_array):
        """根据通道路径生成通道地图."""
        try:
            if not chennal_path_array.markers:
                self.get_logger().warn('没有通道路径数据')
                return None
            if not self.base_map:
                self.get_logger().error('没有基础地图')
                return None

            all_points = []
            for marker in chennal_path_array.markers:
                for point in marker.points:
                    all_points.append((point.x, point.y))

            if not all_points:
                self.get_logger().warn('没有有效的路径点数据')
                return None

            chennal_width = float(self.get_parameter('chennal_width_m').value)

            chennal_map = OccupancyGrid()
            chennal_map.header.stamp = self.get_clock().now().to_msg()
            chennal_map.header.frame_id = self.base_map.header.frame_id
            chennal_map.info = self.base_map.info
            H = self.base_map.info.height
            W = self.base_map.info.width
            ox = self.base_map.info.origin.position.x
            oy = self.base_map.info.origin.position.y

            chennal_map_data = np.full((H, W), 100, dtype=np.uint8)
            resolution = self.base_map.info.resolution

            for marker in chennal_path_array.markers:
                if len(marker.points) < 2:
                    continue

                path_points = []
                for point in marker.points:
                    x = int((point.x - ox) / resolution)
                    y = int((point.y - oy) / resolution)
                    x = max(0, min(x, W-1))
                    y = max(0, min(y, H-1))
                    path_points.append((x, y))

                chennal_width_pixels = int(chennal_width / resolution)

                for i in range(len(path_points) - 1):
                    pt1 = path_points[i]
                    pt2 = path_points[i + 1]

                    temp_img = np.zeros((H, W), dtype=np.uint8)
                    cv2.line(temp_img, pt1, pt2, 1, thickness=chennal_width_pixels)
                    chennal_map_data = np.where(temp_img == 1, 0, chennal_map_data)

            chennal_map.data = chennal_map_data.flatten().tolist()
            return chennal_map

        except Exception as e:
            self.get_logger().error(f'生成通道地图时发生错误: {e}')
            return None

    def _create_chennal_map_inflated(
        self,
        chennal_map: OccupancyGrid,
        inflate_radius_m=None,
    ):
        """膨胀通道地图."""
        if chennal_map is None:
            return None

        inflate_r_m = self.get_inflate_radius_m(inflate_radius_m)
        resolution = chennal_map.info.resolution
        H = chennal_map.info.height
        W = chennal_map.info.width
        chennal_map_data = np.asarray(chennal_map.data, dtype=np.int16).reshape(H, W)
        r_cells = max(0, int(math.ceil(inflate_r_m / resolution)))

        if r_cells > 0:
            chennal_binary = (chennal_map_data == 0).astype(np.uint8)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_cells + 1, 2 * r_cells + 1))
            chennal_binary_eroded = cv2.erode(chennal_binary, k)
            eroded_data = np.where(chennal_binary_eroded == 1, 0, 100).astype(np.int8)
            eroded_map = OccupancyGrid()
            eroded_map.header = chennal_map.header
            eroded_map.info = chennal_map.info
            eroded_map.data = eroded_data.flatten().tolist()
            return eroded_map
        else:
            return chennal_map

    def get_zone_map_list_srv(self, req, res):
        """獲取當前的zone map列表."""
        res.zone_map_list = self.zone_map_list
        return res

    def get_zone_cell_maps(self):
        """获取zone细胞地图列表."""
        return self.zone_cell_maps


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


if __name__ == '__main__':
    main()
