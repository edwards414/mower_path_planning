#!/usr/bin/env python3

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

from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
import numpy as np

from std_msgs.msg import Header
from nav2_simple_commander.robot_navigator import BasicNavigator
# # from costmap_2d import PyCostmap2D
# import costmap_2d
from boustrophedon_coverage.extra.costmap_2d import PyCostmap2D
# from nav2_msgs.msg import Costmap , Path

from geometry_msgs.msg import PoseWithCovarianceStamped

import time
import rclpy

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose, do_transform_point
from rclpy.duration import Duration
from nav_msgs.msg import Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped

class cellIndex:
    def __init__(self,row = None,col = None,theta = None) -> None:
        self.rows = row
        self.cols = col
        self.theta = theta

class CleanRobotCoveragePathPlanning(Node):
    def __init__(self,executor):
        super().__init__('clean_robot_coverage_path_planning')
        self.declare_parameter('SIZE_OF_CELL', 3)
        #create subscriber
        # self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_pose_callback,10)
       #create publisher
        self.path_pub = self.create_publisher(Path, '/coverage_path', 1)

        # self.create_publisher(nav_msgs.msg.Path, '/clean_robot_coverage_path', 1)
        # self.create_publisher(nav_msgs.msg.OccupancyGrid, '/covered_grid', 1)
        #executor
        self.executor = executor
        #object 
        nav = BasicNavigator()
        # self.tf_listener = TransformListener(self.tf_buffer, self)
        #init
        self.initialized = False 
        self.latest_amcl_pose = PoseWithCovarianceStamped()
        self.initOdomPos = Odometry()
        self.robotPos = None
        # 地圖資訊初始化，這裡初始化 global costmap 的型別為 OccupancyGrid
        
        self.costmap2d_ = nav.getGlobalCostmap() #nav2_msg/Costmap
        self.costmap2d_ = PyCostmap2D(self.costmap2d_)

        self.size_x = self.costmap2d_.getSizeInCellsX()
        self.size_y = self.costmap2d_.getSizeInCellsY()
        self.resolution = self.costmap2d_.getResolution()

        self.srcMap_ = np.zeros((self.size_y, self.size_x), dtype=np.uint8)
        for r in range(self.size_y):
            for c in range(self.size_x):
                self.srcMap_[r,c] = self.costmap2d_.getCostXY(c, self.size_y - r - 1)

        if self.srcMap_ is None:
            self.initialized = False
        else:
            self.initialized = True

        # 
        self.neuralizedMats = None

        # 可以在這裡讀取 costmap 的大小，但需要等到第一次收到地圖訊息後才能獲取
        self.cellMat_ = None #cellMat
        self.freeSpace_ = None #freeSpace
        self.neuralizedMat = np.array([]) #neuralizedMats
        self.mapValue = {
            'free': 0,
            'occupied': 100,
            'unknown': 254,
            'obstacle': 255
        }

        #planning data structure
        self.size_of_cell = int(self.get_parameter('SIZE_OF_CELL').value)
        # self.cover_grid_value = int(self.get_parameter('COVER_GRID_VALUE').value)

        #initialize mat
        self.initializeMats()
        self.initializeNeuralMat()
        self.initializeCoveredGrid()

    def initializeMats(self):
        if self.srcMap_ is None:
            return False

        self.getCellMatandFreeSpace()
        self.neuralizedMats = np.zeros((self.cellMat_.shape[0], self.cellMat_.shape[1]),dtype = np.float32)
        self.initializeNeuralMat()
        return True

    def getCellMatandFreeSpace(self):
        size_of_cell = int(self.get_parameter('SIZE_OF_CELL').value)
        FREE_SPACE = self.mapValue['free']  # 0
        LETHAL_OBSTACLE = self.mapValue['unknown'] 
        # srcImg = self.srcMap_
        cell_rows = self.srcMap_.shape[0] // size_of_cell
        cell_cols = self.srcMap_.shape[1] // size_of_cell
        cellMat = np.zeros((cell_rows, cell_cols), dtype=self.srcMap_.dtype)
        freeSpaceVec = []

        for r in range(cell_rows):
            for c in range(cell_cols):
                isFree = True
                for i in range(size_of_cell):
                    for j in range(size_of_cell):
                        map_r = r * size_of_cell + i
                        map_c = c * size_of_cell + j
                        if self.srcMap_[map_r, map_c] != FREE_SPACE:
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
        resolution  = self.costmap2d_.getResolution()
        # 填充header
        self.covered_path_grid_ = OccupancyGrid()
        self.covered_path_grid_.header = Header()
        self.covered_path_grid_.header.frame_id = "map"
        # ROS2中获取当前时间
        self.covered_path_grid_.header.stamp = Clock().now().to_msg()

        # 填充info
        self.covered_path_grid_.info.resolution = resolution
        self.covered_path_grid_.info.width = self.costmap2d_.getSizeInCellsX()
        self.covered_path_grid_.info.height = self.costmap2d_.getSizeInCellsY()

        # 地图原点
        wx, wy = self.costmap2d_.mapToWorld(0, 0)
        self.covered_path_grid_.info.origin.position.x = wx - resolution / 2.0
        self.covered_path_grid_.info.origin.position.y = wy - resolution / 2.0
        self.covered_path_grid_.info.origin.position.z = 0.0
        self.covered_path_grid_.info.origin.orientation.x = 0.0
        self.covered_path_grid_.info.origin.orientation.y = 0.0
        self.covered_path_grid_.info.origin.orientation.z = 0.0
        self.covered_path_grid_.info.origin.orientation.w = 1.0

        # self.covered_path_grid_.data = self.costmap2d_.costmap

        return True
    def mainPlanningLoop(self):
        initPoint, nextPoint , currentPoint = cellIndex(), cellIndex(), cellIndex()
        
        initPoint.theta = 90
        initPoint.rows = self.cellMat_.shape[0] - self.latest_amcl_pose.pose.pose.position.x/self.size_of_cell - 1 #frame map
        initPoint.cols = self.cellMat_.shape[1] - self.latest_amcl_pose.pose.pose.position.y/self.size_of_cell

        pathVec_ = []
        pathVec_.append(initPoint)

        initTheta = initPoint.theta

        c_0 = 50
        e = 0.0 
        v_1 = 0.0
        deltaTheta = 0.0
        lasttheta = initTheta
        PI = 3.14159


        loop = 9000
        thetaVec = [0,45,90,135,180,225,270,315]
        #pos(row,col)
        # thetaPos = [[0,-1],[]]
        for i in range(loop):
                     
            maxIndex = 0
            max_v = -300 

            self.neuralizedMat[currentPoint.rows,currentPoint.cols] = -250
            lasttheta = currentPoint.theta

            for id in range(8):
                deltaTheta = abs(thetaVec[id]-lasttheta)   
                if deltaTheta > 180:
                    deltaTheta = 360 - deltaTheta
                e = 1 - abs(deltaTheta)/180 
                match id:
                    case 0:
                        if(currentPoint.col == self.neuralizedMat.shape[1] - 1):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row,currentPoint.col+1] + c_0 * e
                        break
                    case 1:
                        if(currentPoint.col == self.neuralizedMat_.shape[1] - 1 | self.currentPoint.row == 0):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row - 1,currentPoint.col + 1] + c_0 * e - 200
                        break
                    case 2:
                        if(currentPoint.row == 0):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row - 1,currentPoint.col] + c_0 * e
                        break
                    case 3:
                        if(currentPoint.col == 0 | currentPoint.row == 0):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row - 1,currentPoint.col - 1] + c_0 * e - 200
                        break
                    case 4:
                        if(currentPoint.col == 0):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row ,currentPoint.col - 1] + c_0 * e 
                        break
                    case 5:
                        if(currentPoint.col | currentPoint.row == self.neuralizedMat.shape[0] - 1):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row + 1,currentPoint.col - 1] + c_0 * e - 200
                        break
                    case 6:
                        if(currentPoint.row == self.neuralizedMat.shape[0] - 1):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row + 1,currentPoint.col] + c_0 * e
                        break
                    case 7:
                        if(currentPoint.col == self.neuralizedMat.shape[1] - 1 | currentPoint.row == self.neuralizedMat.shape[0] - 1):
                            v = -100000
                            break
                        v = self.neuralizedMat[currentPoint.row + 1,currentPoint.col + 1] + c_0 * e - 200
                        break
                    case _:
                        break
                if(v > max_v):
                    max_v = v
                    maxIndex = id
                if(max_v == 0 & id > maxIndex):
                    max_v = v;
                    maxIndex = id;
            if max_v <= 0:
                dist = 0.0 
                min_dist = 100000000
                ii=0
                min_index = -1
                for it in self.freeSpace_ :
                    if self.neuralizedMat[it.row, it.col] > 0:
                        if self.Boundingjudge(it.row,it.col):
                            dist = sqrt((currentPoint.row - it.row)**2 + (currentPoint.col - it.col)**2)
                            if dist < min_dist:
                                min_dist = dist
                                min_index = ii
                    ii += 1
                if min_index != -1 & min_dist != 100000000:
                    nextPoint = freeSpaceVec[min_index]
                    currentPoint = nextPoint
                    pathVec_.append(nextPoint)

                    continue
                else:
                    self.get_logger().info("The program has been dead because of the self-locking")
                    self.get_logger().info("The program has gone through %d steps" ,i)
                    break
        match maxIndex:
            case 0 :
                nextPoint.row = currentPoint.row
                nextPoint.col = cruuentPoint.col + 1 
            case 1:
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col + 1
            case 2: 
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col 
            case 3: 
                nextPoint.row = currentPoint.row - 1
                nextPoint.col = currentPoint.col - 1
            case 4: 
                nextPoint.row = currentPoint.row 
                nextPoint.col = currentPoint.col - 1
            case 5: 
                nextPoint.row = currentPoint.row + 1
                nextPoint.col = currentPoint.col - 1
            case 6: 
                nextPoint.row = currentPoint.row + 1
                nextPoint.col = currentPoint.col 
            case 7: 
                nextPoint.row = currentPoint.row + 1    
                nextPoint.col = currentPoint.col + 1
        nextPoint.theta = thetaVec[maxIndex]
        currentPoint = nextPoint
        pathVec_.append(nextPoint)
        path = Path()
        path.header = map_msg.header
        path.header.frame_id = map_msg.header.frame_id or 'map'
        for (x, y,theta) in pathVec_:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = 1.0
            path.poses.append(ps)
        
        self.path_pub.publish(path)

    def Boundingjudge(a,b):
        num = 0
        for i in range(-1,1,1):
            for j in range(-1,1,1):
                if i == 0 & j == 0:
                    continue
                if self.neuralizedMat[a + i, b + j] == -250:
                    num += 1
        if num != 0:
            return True
        else:
            return False




    def getRobotPos(self):
        # 初始化变量
        init_map_pos = None
        point_target = None 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_received = False
        self.robotPosMap = None 
        
        # @staticmethod
        # def tf_listener():
            # timeout = Duration(seconds=0.5)
            # while True:
            #     try:
            #         tf_map_odom = self.tf_buffer.lookup_transform(
            #             'map', 'odom', rclpy.time.Time(),timeout)
                    
            #         # print(tf_map_odom.transform.translation.x,
            #         # tf_map_odom.transform.translation.y,
            #         # tf_map_odom.transform.translation.z)
            #         self.robotPosMap = self.tf_buffer.transform(self.init_map_pos, 'map')
            #         if self.robotPosMap is not None:
            #             print(self.robotPosMap.pose.position.x,self.robotPosMap.pose.position.y)
            #             break
            #     except Exception as e:
            #         self.get_logger().warn(f"TF transform unavailable: {e}")

        # 定义回调函数
        @staticmethod
        def odom_callback(msg):
            self.get_logger().info(f"odom_callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
            odom_pos = PoseStamped()
            odom_pos.header = msg.header
            odom_pos.header.stamp = rclpy.time.Time()
            odom_pos.pose.position.x = msg.pose.pose.position.x
            odom_pos.pose.position.y = msg.pose.pose.position.y
            odom_pos.pose.position.z = msg.pose.pose.position.z
            self.destroy_subscription(odom_sub)

            print(odom_pos.pose.position.x,odom_pos.pose.position.y)
            while True:
                try: 
                    point_target = self.tf_buffer.transform(odom_pos, 'map',timeout=Duration(seconds=5))
                    if point_target is not None:
                        print(point_target.pose.position.x,point_target.pose.position.y)
                        break
                except Exception as e:
                    self.get_logger().warn(f"TF transform unavailable: {e}")
                # self.executor.create_task(tf_listener)

        odom_sub = self.create_subscription(Odometry, '/odom', odom_callback, 10)

def main(args=None):
    print("start main")
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)
    
    Clean_bot_node = CleanRobotCoveragePathPlanning(executor)
    executor.add_node(Clean_bot_node)
    executor.create_task(Clean_bot_node.getRobotPos)
    try:
        executor.spin()
    finally:
        print("finish get robot pos")
        # print(node.init_map_pos.pose.pose.position.x,node.init_map_pos.pose.pose.position.y)
        executor.shutdown()
        Clean_bot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()