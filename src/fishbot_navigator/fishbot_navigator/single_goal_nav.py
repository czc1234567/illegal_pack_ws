import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def main():
    # 1. 初始化 ROS2
    rclpy.init()

    # 2. 实例化 Nav2 操作对象
    navigator = BasicNavigator()

    # --- 关键步骤 A: 设置初始位置 ---
    # 在仿真中，刚启动时 Nav2 不知道机器人在哪。
    # 我们假设机器人是 Gazebo 里从原点 (0,0) 启动的。
    # 如果你不写这一段，你就得手动在 RViz 里点 "2D Pose Estimate"


    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0 # w=1.0 代表方向朝正东
    
    # # 发送初始位置，并等待 Nav2 系统完全启动
    # print("正在设置初始位置并等待 Nav2 启动...")
    # navigator.setInitialPose(initial_pose)

    
    # navigator.waitUntilNav2Active(localizer=None)
    # print("Nav2 已就绪！")

    # --- 关键步骤 B: 设置目标点 ---
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg() 
    
    # ！！！请根据你的地图修改这里的坐标！！！
    # 你可以在 RViz 里用鼠标点一下大概位置，看左下角的坐标值
    goal_pose.pose.position.x = -12.0  # 向前走 2 米
    goal_pose.pose.position.y = 6.2  # 向左走 0.5 米
    goal_pose.pose.orientation.w = 1.0

    print(f"正在前往目标点: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})...")
    navigator.goToPose(goal_pose)

    # --- 关键步骤 C: 监控任务状态 ---
    while not navigator.isTaskComplete():
        # 获取反馈（可选：打印剩余距离等）
        feedback = navigator.getFeedback()
        # 这里可以加一些处理逻辑，比如如果时间太长就取消任务
        # print(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
        
    # --- 关键步骤 D: 处理结果 ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('导航成功！到达目的地。')
    elif result == TaskResult.CANCELED:
        print('导航被取消。')
    elif result == TaskResult.FAILED:
        print('导航失败！可能是路被堵住了。')

    # 关闭
    # navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
