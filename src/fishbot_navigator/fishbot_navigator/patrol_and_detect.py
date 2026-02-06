#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
import time
import math

# ==========================================
# 辅助节点：监听报警 + 读取 RViz 标记
# ==========================================
class AlertListener(Node):
    def __init__(self):
        super().__init__('alert_listener')
        self.alert_received = False
        self.known_obstacles = [] 
        
        self.sub_alert = self.create_subscription(
            Bool, '/detection/alert', self.listener_callback, 10)
            
        self.sub_markers = self.create_subscription(
            MarkerArray, '/detection/result_map', self.marker_callback, 10)
        
        self.get_logger().info("🎧 守护者已就绪：监听报警信号 & 扫描地图标记...")

    def listener_callback(self, msg):
        if msg.data:
            self.alert_received = True

    def marker_callback(self, msg):
        for marker in msg.markers:
            if marker.ns == "permanent_violations":
                obs_x = marker.pose.position.x
                obs_y = marker.pose.position.y
                self.add_obstacle_if_new(obs_x, obs_y)

    def add_obstacle_if_new(self, x, y):
        new_obs = (x, y)
        is_known = False
        for obs in self.known_obstacles:
            dist = math.sqrt((obs[0]-new_obs[0])**2 + (obs[1]-new_obs[1])**2)
            if dist < 0.5:
                is_known = True
                break
        
        if not is_known:
            self.known_obstacles.append(new_obs)
            self.get_logger().info(f"📝 地图更新：发现新违停障碍物 ({x:.2f}, {y:.2f})")

# ==========================================
# 辅助函数
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

def check_if_goal_blocked(goal_pose, obstacles, threshold=1.5):
    gx = goal_pose.pose.position.x
    gy = goal_pose.pose.position.y
    for ox, oy in obstacles:
        dist = math.sqrt((gx - ox)**2 + (gy - oy)**2)
        if dist < threshold:
            return True, (ox, oy)
    return False, None

# ==========================================
# 主逻辑
# ==========================================
def main():
    rclpy.init()
    navigator = BasicNavigator()
    alert_node = AlertListener()

    print("🚀 跳过 Nav2 状态检查，直接启动导航逻辑...")
    # navigator.waitUntilNav2Active(localizer=None) 

    waypoints = []
    
    # 室内/室外坐标点 (请根据需要取消注释或修改)

    #   室内场景坐标点
    # # 示例点 1
    # waypoints.append(create_pose(navigator, -1.4, 5.0))
    # # 示例点 2
    # waypoints.append(create_pose(navigator, -13.1, 6.3))
    # # 示例点 3
    # waypoints.append(create_pose(navigator, -13.5, 1.35))
    # # 示例点 4 (回到原点附近)
    # waypoints.append(create_pose(navigator, -1.6, 0.0))
    
    # 室外示例
    # waypoints.append(create_pose(navigator, 5.7, -68.6)) 
    # waypoints.append(create_pose(navigator, 105.7, -79.0)) 
    # waypoints.append(create_pose(navigator, 104.5, -91.2))
    # waypoints.append(create_pose(navigator, 5.7, -79.2))   
    
      
    # # 室外示例2
    # waypoints.append(create_pose(navigator, 5.7, -68.6)) 
    # waypoints.append(create_pose(navigator, 105.7, -79.0)) 
    # waypoints.append(create_pose(navigator, 103.5, -91.2))
    # waypoints.append(create_pose(navigator, 106, -64))       
    # waypoints.append(create_pose(navigator, 5, -53))      

    # 室外示例3
    waypoints.append(create_pose(navigator, 5.7, -69.6)) 
    waypoints.append(create_pose(navigator, 105.7, -81.0))
    waypoints.append(create_pose(navigator, 106, -64)) 
    waypoints.append(create_pose(navigator, -70.64, -45.30))
    waypoints.append(create_pose(navigator, -84.76, -60.08))
    waypoints.append(create_pose(navigator, 105.7, -79.1))
    waypoints.append(create_pose(navigator, 5.20, -53.20))

    # 策略一：障碍物避让半径 (米)
    OBSTACLE_RADIUS = 5

    # 策略二：原地打转检测参数
    STALL_TIMEOUT = 10.0   # 如果在 10秒内
    STALL_DISTANCE = 0.2   # 移动距离小于 0.2米 -> 判定为卡死/遇到未知障碍物

    print(f"📋 开始执行巡检任务，共 {len(waypoints)} 个站点。(已移除超时限制)")
    time.sleep(1.0) 

    # ========================================================
    # 使用 try...except 捕捉 Ctrl+C
    # ========================================================
    try:
        for index, goal_pose in enumerate(waypoints):
            
            # 刷新障碍物信息
            rclpy.spin_once(alert_node, timeout_sec=0.1)
            
            target_x = goal_pose.pose.position.x
            target_y = goal_pose.pose.position.y
            3.

            # =========================================================
            # 【策略一】出发前检查：目标点是否被标记占据？
            # =========================================================
            is_blocked, obs_pos = check_if_goal_blocked(goal_pose, alert_node.known_obstacles, OBSTACLE_RADIUS)
            if is_blocked:
                print(f"\n🚫 站点 {index + 1} ({target_x:.1f}, {target_y:.1f}) 被违停车辆 {obs_pos} 占据！")
                print("🏃‍♂️ [主动避让] 跳过该点，直接前往下一站。")
                continue 

            print(f"\n>>> 正在前往第 {index + 1} 个站点 (x={target_x}, y={target_y}) >>>")
            
            # 发送导航指令
            navigator.goToPose(goal_pose)

            # 初始化卡死检测变量
            last_check_time = time.time()
            last_check_pos = None 
            
            # 任务循环
            while not navigator.isTaskComplete():
                rclpy.spin_once(alert_node, timeout_sec=0.1)

                # =========================================================
                # 【策略二】行进中检查：是否原地打转（遇到未知障碍物）？
                # =========================================================
                current_time = time.time()
                # 每隔一段时间检查一次位置变化
                if (current_time - last_check_time) > STALL_TIMEOUT:
                    # 获取当前反馈（机器人位置）
                    feedback = navigator.getFeedback()
                    if feedback:
                        curr_x = feedback.current_pose.pose.position.x
                        curr_y = feedback.current_pose.pose.position.y
                        
                        if last_check_pos is not None:
                            # 计算这段时间走了多远
                            dist_moved = math.sqrt((curr_x - last_check_pos[0])**2 + (curr_y - last_check_pos[1])**2)
                            
                            # 如果时间过去了，但没怎么动 -> 说明被未知障碍物挡住了，或者在原地转圈
                            if dist_moved < STALL_DISTANCE:
                                print(f"\n🧱 检测到原地打转/卡死 (10秒仅移动 {dist_moved:.2f}m)！")
                                print(f"🤔 可能是被未知障碍物（非违停车辆）阻挡。")
                                print(f"🏃‍♂️ [被动避让] 放弃当前站点 {index + 1}，前往下一站。")
                                navigator.cancelTask()
                                break 
                        
                        # 更新记录
                        last_check_pos = (curr_x, curr_y)
                        last_check_time = current_time

                # --- 报警中断逻辑 ---
                if alert_node.alert_received:
                    print("\n🛑【紧急停车】检测到违停！正在处理...")
                    navigator.cancelTask()
                    
                    # 等待时保持 Spin
                    wait_time = 5
                    for i in range(wait_time, 0, -1):
                        rclpy.spin_once(alert_node, timeout_sec=0.1) 
                        print(f"⏳ 处理违停中... {i}s (正在更新地图信息)", end='\r')
                        time.sleep(0.9) 
                    
                    print("\n✅ 处理完毕。")
                    alert_node.alert_received = False
                    
                    # ====================================================
                    # 【安全重启逻辑】清除地图 + 等待刷新
                    # ====================================================
                    
                    # print("🧹 正在清除局部地图缓存...")
                    # navigator.clearLocalCostmap()
                    
                    print("👀 等待雷达刷新环境 (5s)...")
                    for _ in range(50): 
                        rclpy.spin_once(alert_node, timeout_sec=0.1)
                        time.sleep(0.1)

                    # 恢复前，再次检查
                    is_blocked, obs_pos = check_if_goal_blocked(goal_pose, alert_node.known_obstacles, OBSTACLE_RADIUS)
                    
                    if is_blocked:
                        print(f"\n🚫 刚才检测到的违停车辆 {obs_pos} 距离目标太近！")
                        print(f"🏃‍♂️ [动态避让] 放弃当前站点 {index + 1}，直接前往下一站。")
                        break 

                    print(f"🔄 恢复巡逻，继续前往第 {index + 1} 个站点...")
                    navigator.goToPose(goal_pose)

                    # 恢复后，重置卡死检测器，防止误判
                    last_check_time = time.time()
                    last_check_pos = None
                    continue

            # --- 结果处理 ---
            result = navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                print(f"🎉 到达站点 {index + 1}。")
                time.sleep(1.0)
                
            elif result == TaskResult.CANCELED:
                print(f"⚠️ 站点 {index + 1} 导航被取消。")
                pass 
                
            elif result == TaskResult.FAILED:
                print(f"❌ 站点 {index + 1} 规划失败 (无路可走)。跳过。")
                continue

    # ========================================================
    # 【精简版】捕捉 Ctrl+C，仅取消任务
    # ========================================================
    except KeyboardInterrupt:
        print("\n\n🛑 收到手动终止信号 (Ctrl+C)！")
        print("⚠️ 正在请求 Nav2 取消任务...")
        
        # 只调用 Nav2 的标准取消接口
        # Nav2 收到 Cancel 后，底层控制器会自动让机器狗停下来
        navigator.cancelTask()
        
        print("✅ 任务已取消，程序即将退出。")

    finally:
        print("\n🏁 程序退出。")
        alert_node.destroy_node()
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()