#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import time

# ==========================================
# 辅助节点：专门用于监听“违停报警”信号
# ==========================================
class AlertListener(Node):
    def __init__(self):
        super().__init__('alert_listener')
        self.alert_received = False
        
        # 订阅检测节点发出的报警信号
        self.subscription = self.create_subscription(
            Bool,
            '/detection/alert',
            self.listener_callback,
            10
        )
        self.get_logger().info("🎧 报警监听器已就绪，等待信号...")

    def listener_callback(self, msg):
        if msg.data:
            self.alert_received = True
            self.get_logger().warn('🚨 收到违停报警信号！请求停车！')

# ==========================================
# 辅助函数：创建坐标点
# ==========================================
def create_pose(navigator, x, y, z_w=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = float(z_w)
    pose.pose.orientation.z = 0.0
    return pose

# ==========================================
# 主逻辑
# ==========================================
def main():
    # 1. 初始化 ROS2
    rclpy.init()
    
    # 2. 实例化对象
    navigator = BasicNavigator()
    alert_node = AlertListener()

    # 3. 【关键】跳过 Nav2 检查，直接启动
    # 因为你的环境会报 service not available，所以我们强制跳过
    print("🚀 跳过 Nav2 状态检查，直接启动导航逻辑...")
    # navigator.waitUntilNav2Active(localizer=None) 

    # 4. 定义巡逻路径点 
    # ！！！请务必修改为你 RViz 地图上真实的坐标！！！
    waypoints = []
    
    # 示例点 1 
    waypoints.append(create_pose(navigator, -1.4, 5.0)) 
    # 示例点 2 
    waypoints.append(create_pose(navigator, -12.0, 6.2)) 
    # 示例点 3
    waypoints.append(create_pose(navigator, -14.0, 1.45))
    # 示例点 4 (回到原点附近)
    waypoints.append(create_pose(navigator, 0.0, 0.0))

    print(f"📋 开始执行巡检任务，共 {len(waypoints)} 个站点。")
    time.sleep(1.0) # 稍微等一下让打印输出

    # 5. 核心巡逻循环
    for index, goal_pose in enumerate(waypoints):
        print(f"\n>>> 正在前往第 {index + 1} 个站点 (x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}) >>>")
        
        # 发送导航指令
        navigator.goToPose(goal_pose)
        
        # 任务循环：只要没到达，就一直检查有没有报警
        while not navigator.isTaskComplete():
            
            # --- A. 检查报警信号 ---
            # spin_once 会查看 alert_node 有没有收到消息
            rclpy.spin_once(alert_node, timeout_sec=0.1)

            # --- B. 如果收到报警，执行中断逻辑 ---
            if alert_node.alert_received:
                print("\n🛑【紧急停车】检测到违停！正在处理...")
                
                # 1. 立即取消当前导航，让狗停下
                navigator.cancelTask()
                
                # 2. 原地等待 (留时间给检测节点拍照、发语音、上传数据)
                # 这里设置 5-8 秒比较合适，等语音播报完
                wait_time = 10
                for i in range(wait_time, 0, -1):
                    print(f"⏳ 处理违停中... {i}s", end='\r')
                    time.sleep(1.0)
                print("\n✅ 处理完毕。")
                
                # 3. 复位标志位，防止死循环
                alert_node.alert_received = False
                
                # 4. 【关键】恢复导航 (Resume)
                # 重新发送刚才没走完的那个目标点
                print(f"🔄 恢复巡逻，继续前往第 {index + 1} 个站点...")
                navigator.goToPose(goal_pose)
                
                # 跳过本次循环剩下的部分，直接进入下一次 while 检查
                continue

            # --- C. 获取反馈 (可选) ---
            # feedback = navigator.getFeedback()
            # if feedback:
            #     print(f'距离: {feedback.distance_remaining:.2f}m', end='\r')

        # --- D. 单点任务结束处理 ---
        result = navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            print(f"🎉 到达站点 {index + 1}。")
            # 到达站点后可以稍微停顿一下
            time.sleep(1.0)
            
        elif result == TaskResult.CANCELED:
            # 如果是因为我们手动 cancel 导致的，通常会被上面的 Resume 逻辑覆盖
            # 但如果因为其他原因被取消，打印日志
            print(f"⚠️ 站点 {index + 1} 导航被取消。")
            
        elif result == TaskResult.FAILED:
            print(f"❌ 站点 {index + 1} 导航失败 (可能是路堵了)。跳过，去下一个点。")
            continue

    print("\n🏁 所有站点巡检完毕！任务结束。")
    
    # 清理资源
    alert_node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()