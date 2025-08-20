#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
import numpy as np

from std_msgs.msg import Header
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseWithCovarianceStamped

import time
import rclpy

class CleanRobotCoveragePathPlanning(Node,BasicNavigator):
    def __init__(self):
        super().__init__('clean_robot_coverage_path_planning')
        self.declare_parameter('SIZE_OF_CELL', 3)
        #create subscriber
        # self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.initializeMapParam, 10)
        self.create_subscription(OccupancyGrid, '', self.initializeMapParam, 10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)
        #create publisher
        self.create_publisher(nav_msgs.msg.Path, '/clean_robot_coverage_path', 1)
        self.create_publisher(nav_msgs.msg.OccupancyGrid, '/covered_grid', 1)
        
        #init
        self.initialized = False 
        self.latest_amcl_pose = None
        #map info
        self.costmap2d_ = None
        self.size_x = None
        self.size_y = None
        self.resolution = None
        self.srcMap_ = None

        # 可以在這裡讀取 costmap 的大小，但需要等到第一次收到地圖訊息後才能獲取
        self.cellMat_ = None #cellMat
        self.freeSpace_ = None #freeSpace
        self.neuralizedMat = None #neuralizedMats
        self.mapValue = {
            'free': 0,
            'occupied': 100,
            'unknown': 254,
            'obstacle': 255
        }

        #planning data structure
        self.size_of_cell = int(self.get_parameter('SIZE_OF_CELL').value)
        self.cover_grid_value = int(self.get_parameter('COVER_GRID_VALUE').value)

        #initialize mat
        self.initializeMat()
        self.initializeNeuralMat()
        self.initializeCoveredGrid()

    def initializeMapParam(self):
        self.costmap2d_ = self.getLocalCostmap()
        self.size_x = self.costmap2d_.info.width
        self.size_y = self.costmap2d_.info.height
        self.resolution = self.costmap2d_.info.resolution
        self.srcMap_ = np.array(self.costmap2d_.data).reshape(self.size_y, self.size_x)
        if self.srcMap_ is None:
            self.initialized = False
        else:
            self.initialized = True

    def initializeMat(self):
        if self.costmap_2d is None:
            return False
        self.getCellMatandFreeSpace()
        
        self.neuralizedMat = np.array(self.cellMat_.shape[0], self.cellMat_.shape[1],dtype = np.float32 )
        self.initializeNeuralMat(neuralizedMat)
        return True

    def getCellMatandFreeSpace(self):
        size_of_cell = int(self.get_parameter('SIZE_OF_CELL').value)
        FREE_SPACE = self.mapValue['free']  # 0
        LETHAL_OBSTACLE = self.mapValue['unknown'] 
        srcImg = self.srcMap_
        cell_rows = srcImg.shape[0] // size_of_cell
        cell_cols = srcImg.shape[1] // size_of_cell
        cellMat = np.zeros((cell_rows, cell_cols), dtype=srcImg.dtype)
        freeSpaceVec = []

        for r in range(cell_rows):
            for c in range(cell_cols):
                isFree = True
                for i in range(size_of_cell):
                    for j in range(size_of_cell):
                        map_r = r * size_of_cell + i
                        map_c = c * size_of_cell + j
                        if srcImg[map_r, map_c] != FREE_SPACE:
                            isFree = False
                            break
                    if not isFree:
                        break
                if isFree:
                    # cellIndex: row, col, theta
                    freeSpaceVec.append({'row': r, 'col': c, 'theta': 0})
                    cellMat[r, c] = FREE_SPACE
                else:
                    cellMat[r, c] = LETHAL_OBSTACLE
        self.cellMat_ = cellMat
        self.freeSpace_ = freeSpaceVec
        print(f"freespace size: {len(freeSpaceVec)}")
  
    def initializeNeuralMat(self):
        for i in range(self.neuralizedMat.shape[0]):
            for j in range(self.neuralizedMat.shape[1]):
                if self.cellMat_[i,j] == self.mapValue['obstacle']:
                    self.neuralizedMat[i,j] = -100000.0
                else:
                    self.neuralizedMat[i,j] = 50.0/j
         
   #function initializeCoveredGrid
   # 功能蒐集地圖資訊
   # return bool 

    def initializeCoveredGrid(self) -> bool: 
        # 获取分辨率
        resolution  = self.costmap2d_.info.resolution
        # 填充header
        self.covered_path_grid_ = OccupancyGrid()
        self.covered_path_grid_.header = Header()
        self.covered_path_grid_.header.frame_id = "map"
        # ROS2中获取当前时间
        self.covered_path_grid_.header.stamp = Clock().now().to_msg()

        # 填充info
        self.covered_path_grid_.info.resolution = resolution
        self.covered_path_grid_.info.width = self.costmap2d_.info.width
        self.covered_path_grid_.info.height = self.costmap2d_.info.height

        # 地图原点
        wx, wy = self.costmap2d_.mapToWorld(0, 0)
        self.covered_path_grid_.info.origin.position.x = wx - resolution / 2.0
        self.covered_path_grid_.info.origin.position.y = wy - resolution / 2.0
        self.covered_path_grid_.info.origin.position.z = 0.0
        self.covered_path_grid_.info.origin.orientation.x = 0.0
        self.covered_path_grid_.info.origin.orientation.y = 0.0
        self.covered_path_grid_.info.origin.orientation.z = 0.0
        self.covered_path_grid_.info.origin.orientation.w = 1.0

        self.covered_path_grid_.data = self.costmap2d_.data

        return True
    def mainPlanningLoop(self):
        initPoint, nextPoint , currentPoint = None, None, None

        initPoint.theta = 0

    
    
    def GetPathInROS(self):
        pass

    def GetBorderTrackingPathInROS(self):
        pass

    def GetBorderTrackingPathInCV(self):
        pass

    def SetCoveredGrid(self):
        pass

    def PublishGrid(self):
        pass
    
    def GetPathInCV(self):
        pass 

    def PublishCoveragePath(self):
        pass 

    def PublishPath(self):
        pass

    def cellContainsPoint(self,cell,point):
        pass
    
   def getRobotPos(self) -> geometry_msgs.msg.Pose:
        """获取机器人当前位置 - 只订阅一次"""
        # 重置位置
        self.latest_amcl_pose = None
        
        # 创建临时订阅
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.amcl_pose_callback, 
            10
        )
        
        # 等待接收到位置消息
        timeout = 2.0  # 最多等待2秒
        start_time = time.time()
        while self.latest_amcl_pose is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.latest_amcl_pose is not None:
            self.get_logger().info("成功获取机器人位置")
            return self.latest_amcl_pose
        else:
            self.get_logger().warn("未能在规定时间内获取到 /amcl_pose 消息，返回None")
            # 确保清理订阅
            if self.amcl_subscription is not None:
                self.destroy_subscription(self.amcl_subscription)
                self.amcl_subscription = None
            return None

        
    

def main(args=None):
    rclpy.init(args=args)
    node = CleanRobotCoveragePathPlanning()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()