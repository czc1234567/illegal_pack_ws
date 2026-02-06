import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math

def create_quaternion(yaw_degrees):
    """
    辅助函数：将角度转换为四元数   map 小狗的初始姿态下 朝向正南 (0 度)
    yaw_degrees: 角度 (0=正南, 90=正东, 180=正北, -90=正西)
    """
    yaw_radians = math.radians(yaw_degrees)
    # 绕 Z 轴旋转的简化公式
    z = math.sin(yaw_radians / 2.0)
    w = math.cos(yaw_radians / 2.0)
    return z, w

def main():
    # 1. 初始化 ROS2
    rclpy.init()

    # 2. 实例化 Nav2 操作对象
    navigator = BasicNavigator()

    # --- 关键步骤 A: 设置初始位置 ---
    # (如果机器人已经在地图上有定位，这一段保持注释即可)
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0 
    # navigator.setInitialPose(initial_pose)
    # navigator.waitUntilNav2Active(localizer=None)

    # --- 关键步骤 B: 设置目标点 ---
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg() 
    
    # 1. 设置目标位置 (根据您之前的需求 x=-12.0)
    goal_pose.pose.position.x = -12.0  
    goal_pose.pose.position.y = 6.2  
    goal_pose.pose.position.z = 0.0

    # 2. 设置朝向：正西 (180度)
    # 方法一：直接写死四元数 (正西是 z=1.0, w=0.0)
    # goal_pose.pose.orientation.z = 1.0
    # goal_pose.pose.orientation.w = 0.0
    
    # 方法二：使用辅助函数 (推荐，更直观)
    target_yaw = -90  # 正西方
    z, w = create_quaternion(target_yaw)
    goal_pose.pose.orientation.z = z
    goal_pose.pose.orientation.w = w

    print(f"正在前往目标点: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y}), 朝向: {target_yaw}度 (正西)...")
    navigator.goToPose(goal_pose)

    # --- 关键步骤 C: 监控任务状态 ---
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # 可以在这里打印剩余距离
        # if feedback:
        #     print(f'距离目标还有: {feedback.distance_remaining:.2f} 米')
        
    # --- 关键步骤 D: 处理结果 ---
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('导航成功！到达目的地。')
    elif result == TaskResult.CANCELED:
        print('导航被取消。')
    elif result == TaskResult.FAILED:
        print('导航失败！可能是路被堵住了。')

    # 退出
    exit(0)

if __name__ == '__main__':
    main()