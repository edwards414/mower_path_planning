import math
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def _euler_to_quaternion(roll, pitch, yaw):
    """
    將歐拉角轉換為四元數（x, y, z, w）
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return (qx, qy, qz, qw)

def _cal_two_point_orientation(x1, y1, x2, y2):
    """
    計算兩點之間的朝向（歐拉角yaw），並返回對應的四元數
    """
    dx = x2 - x1
    dy = y2 - y1
    yaw = math.atan2(dy, dx)
    return _euler_to_quaternion(0, 0, yaw)

def _transform_coverage_path_points(points: list,map_header: Header) -> Path:
    """
    將路徑點轉換為Path消息
    """
    coverage_path = Path()
    coverage_path.header = map_header
    coverage_path.header.frame_id = 'map'

    for idx, (x, y) in enumerate(points):
        goal_pose = PoseStamped()
        goal_pose.header = coverage_path.header
        goal_pose.header.stamp = map_header.stamp #使用node的clock
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0

        # 計算朝向
        if idx < len(points) - 1:
            x2, y2 = points[idx + 1]
            qx, qy, qz, qw = _cal_two_point_orientation(x, y, x2, y2)
        elif idx > 0:
            x2, y2 = points[idx - 1]
            qx, qy, qz, qw = _cal_two_point_orientation(x2, y2, x, y)
        else:
            # 只有一個點，朝向正前
            qx, qy, qz, qw = _euler_to_quaternion(0, 0, 0)

        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        coverage_path.poses.append(goal_pose)
    return coverage_path

def _validate_maps_compatibility(self, map1: OccupancyGrid, map2: OccupancyGrid) -> bool:
    """驗證兩個地圖是否兼容（相同的分辨率、尺寸和原點）"""
    info1, info2 = map1.info, map2.info
    
    # 檢查分辨率
    if abs(info1.resolution - info2.resolution) > 1e-6:
        return False
        
    # 檢查尺寸
    if info1.width != info2.width or info1.height != info2.height:
        return False
        
    # 檢查原點
    if (abs(info1.origin.position.x - info2.origin.position.x) > 1e-6 or
        abs(info1.origin.position.y - info2.origin.position.y) > 1e-6):
        return False
            
    return True



class val():
    def __init__(self):
        self.x = None   
        self.y = None
        self.qx = None
        self.qy = None 
        self.qz = None
        self.qw = None
    def is_empty(self):
        return self.x == None and self.y == None and self.qx == None and self.qy == None and self.qz == None and self.qw == None
