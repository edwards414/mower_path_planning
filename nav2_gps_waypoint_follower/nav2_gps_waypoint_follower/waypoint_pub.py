from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy
from rclpy.duration import Duration
import time

def main():
    rclpy.init()

    nav = BasicNavigator()

    # 設定四個點，形成一個長寬30米的矩形
    # 假設左下角為 (0, 0)，逆時針方向
    width = 4.0
    points = [
        (0.0, 0.0),
        (width, 0.0),
        (width, width),
        (0.0, width)
    ]

    goal_poses = []
    for (x, y) in points:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_pose.pose.orientation.z = 0.0
        goal_poses.append(goal_pose)

    nav_start = nav.get_clock().now()
    print("開始發佈waypoint")
    print(len(goal_poses))
    goal_poses = goal_poses[::-1]
    nav.followWaypoints(goal_poses)

    i = 0
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"當前正在執行第 {feedback.current_waypoint + 1} 個航點")
            if feedback.current_waypoint != i:
                i = feedback.current_waypoint
                print(f"已到達航點 {i}, 前往下一個航點...")
        now = nav.get_clock().now()
         # 設定超時取消 (10分鐘)
        if now - nav_start > Duration(seconds=600.0):
            print("導航超時，取消任務")
            nav.cancelTask()
            break
        time.sleep(1)

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    
    # nav.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
