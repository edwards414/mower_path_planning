import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def simplify_path(poses, distance_threshold):
    """
    使用Douglas-Peucker算法简化路径
    """
    if len(poses) < 3:
        return poses
        
    points = [(p.pose.position.x, p.pose.position.y) for p in poses]
    simplified_indices = douglas_peucker(points, distance_threshold)
    return [poses[i] for i in simplified_indices]

def douglas_peucker( points, epsilon):
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
        dist = point_to_line_distance(points[i], start, end)
        if dist > max_dist:
            max_dist = dist
            max_index = i
    
    # 如果最大距离大于阈值，递归处理
    if max_dist > epsilon:
        # 递归处理前半段和后半段
        left_indices = douglas_peucker(points[:max_index + 1], epsilon)
        right_indices = douglas_peucker(points[max_index:], epsilon)
        
        # 合并结果，注意避免重复
        result = left_indices + [max_index + i for i in right_indices[1:]]
        return result
    else:
        # 如果最大距离小于阈值，只保留起点和终点
        return [0, len(points) - 1]

def point_to_line_distance(point, line_start, line_end):
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
#定時器發佈機器人路徑