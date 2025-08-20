from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, PoseStamped

# rclpy.init()
# nav = BasicNavigator()




# map = nav.getLocalCostmap()
# # nav.clearGlobalCostmap()




# print (map)

timeout = Duration(seconds=1.0)

tf_buffer = Buffer()
is_ok = tf_buffer.canTransform(
    target_frame = 'map', 
    source_frame = 'base_link', 
    time = rclpy.time.Time(), 
    timeout = timeout)
if is_ok:
    print("can transform")
else:
    print("can not transform")








