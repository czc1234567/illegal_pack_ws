import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

# 辅助函数：方便快速创建坐标点
def create_pose(navigator, x, y, z_w=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = z_w
    pose.pose.orientation.z = 0.0
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 1. 等待 Nav2 启动
    # 如果你已经在 RViz 里设置了初始位置，这行代码会直接通过
    # 如果没设置，它会一直在这里等待
    print("等待 Nav2 系统就绪...")
    navigator.waitUntilNav2Active()

    # 2. 定义巡逻路径点 (列表)
    # 请根据你的地图修改这里的坐标 (x, y)
    # 建议在 RViz 里用 "Publish Point" 先点一下看看坐标
    goal_poses = []
    
    # 第 1 个点
    goal_poses.append(create_pose(navigator, 2.0, -0.2))
    # 第 2 个点
    goal_poses.append(create_pose(navigator, 0.0, 2.0))
    # 第 3 个点
    goal_poses.append(create_pose(navigator, -2.0, 0.0))
    # 第 4 个点 (回到原点)
    goal_poses.append(create_pose(navigator, 0.0, 0.0))

    # 3. 开始多点巡航
    print(f"开始巡航，共有 {len(goal_poses)} 个目标点。")
    # 核心 API：followWaypoints
    navigator.followWaypoints(goal_poses)

    # 4. 监控任务进度
    i = 0
    while not navigator.isTaskComplete():
        ################################################
        # 获取反馈：当前正在去第几个点
        ################################################
        feedback = navigator.getFeedback()
        if feedback:
            current_waypoint = feedback.current_waypoint
            print(f'正在前往第 {current_waypoint + 1} 个点...', end='\r')
        
        # 稍微延时，防止 CPU 占用过高
        time.sleep(1.0)

    # 5. 处理结果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("\n巡逻任务完成！所有点都已到达。")
    elif result == TaskResult.CANCELED:
        print("\n任务被取消！")
    elif result == TaskResult.FAILED:
        print("\n任务失败！某个点无法到达。")

    # ---------------------------------------------------------
    # 【重点】不要关闭 Nav2，这样你可以继续在 RViz 操作
    # navigator.lifecycleShutdown() 
    # ---------------------------------------------------------
    
    print("脚本退出，但 Nav2 保持运行。")
    exit(0)

if __name__ == '__main__':
    main()