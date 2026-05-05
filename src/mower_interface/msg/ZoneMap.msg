# ZoneMap 消息定義
# 用於發布區域地圖信息

# 標準消息頭
std_msgs/Header header

# 區域ID
int32 zone_id

# 原始遮罩地圖
nav_msgs/OccupancyGrid mask_map

# 膨脹後的遮罩地圖
nav_msgs/OccupancyGrid mask_map_inflated

#路徑
nav_msgs/Path path

# 覆蓋分割點
geometry_msgs/Pose[] coverage_split_points
