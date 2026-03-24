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
import json
import sys

# ==========================================
# 综合节点：管理参数 + 监听报警 + 读取 RViz 标记
# ==========================================
class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_navigator_node')
        
        # 1. 声明并读取参数 (与 YAML 文件对应)
        self.declare_parameter('waypoints_json', '[]')
        self.declare_parameter('obstacle_radius', 5.0)
        self.declare_parameter('stall_timeout', 20.0)
        self.declare_parameter('stall_distance', 0.2)

        self.waypoints_json_str = self.get_parameter('waypoints_json').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        self.stall_distance = self.get_parameter('stall_distance').value
        
        # 解析 JSON 航点
        try:
            self.parsed_waypoints = json.loads(self.waypoints_json_str)
        except Exception as e:
            self.get_logger().error(f"❌ 航点 JSON 解析失败，请检查 yaml 格式: {e}")
            self.parsed_waypoints = []

        # 2. 初始化订阅状态
        self.alert_received = False
        self.known_obstacles = [] 
        
        self.sub_alert = self.create_subscription(Bool, '/detection/alert', self.listener_callback, 10)
        self.sub_markers = self.create_subscription(MarkerArray, '/detection/result_map', self.marker_callback, 10)
        
        self.get_logger().info(f"⚙️ 载入配置：避障半径 {self.obstacle_radius}m | 打转超时 {self.stall_timeout}s")
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
def main(args=None):
    if args is None:
        args = sys.argv
        
    # =========================================================
    # ⚠️ 【请修改这里】：配置你的 default_yaml_path 绝对路径
    # =========================================================
    default_yaml_path = 'weight/patrol_params.yaml'
    
    if '--ros-args' not in args:
        args.extend(['--ros-args', '--params-file', default_yaml_path])

    rclpy.init(args=args)
    patrol_node = PatrolNode()
    navigator = BasicNavigator()

    print("🚀 跳过 Nav2 状态检查，直接启动导航逻辑...")
    # navigator.waitUntilNav2Active(localizer=None) 

    # 从节点中提取配置好的参数
    waypoints = patrol_node.parsed_waypoints
    OBSTACLE_RADIUS = patrol_node.obstacle_radius
    STALL_TIMEOUT = patrol_node.stall_timeout
    STALL_DISTANCE = patrol_node.stall_distance

    if not waypoints:
        print("❌ 航点列表为空，退出程序。")
        return

    print(f"📋 开始执行巡检任务，共 {len(waypoints)} 个站点。(已区分任务点与途径点)")
    time.sleep(1.0) 

    try:
        for index, pt_info in enumerate(waypoints):
            
            # 刷新障碍物信息
            rclpy.spin_once(patrol_node, timeout_sec=0.1)
            
            # 提取坐标和属性
            target_x = float(pt_info["x"])
            target_y = float(pt_info["y"])
            point_type = pt_info.get("type", "via")
            is_task_point = (point_type == "task")  # 布尔值：是否为任务点
            
            goal_pose = create_pose(navigator, target_x, target_y)

            # =========================================================
            # 【策略一】途径点检查：目标点是否被标记占据？任务点无视。
            # =========================================================
            if not is_task_point:
                is_blocked, obs_pos = check_if_goal_blocked(goal_pose, patrol_node.known_obstacles, OBSTACLE_RADIUS)
                if is_blocked:
                    print(f"\n🚫 途径站点 {index + 1} ({target_x:.1f}, {target_y:.1f}) 被违停车辆 {obs_pos} 占据！")
                    print("🏃‍♂️ [主动避让] 跳过该点，直接前往下一站。")
                    continue 
            else:
                print(f"\n🎯 站点 {index + 1} 是【任务点】，无视违停占据检查。")

            print(f"\n>>> 正在前往第 {index + 1} 个站点 (x={target_x}, y={target_y}, 类型={point_type}) >>>")
            
            navigator.goToPose(goal_pose)

            # 初始化卡死检测变量
            last_check_time = time.time()
            last_check_pos = None 
            
            # 任务循环
            while not navigator.isTaskComplete():
                rclpy.spin_once(patrol_node, timeout_sec=0.1)

                # =========================================================
                # 【策略二】行进中检查：途径点启用打转超时，任务点禁用
                # =========================================================
                current_time = time.time()
                if not is_task_point:
                    if (current_time - last_check_time) > STALL_TIMEOUT:
                        feedback = navigator.getFeedback()
                        if feedback:
                            curr_x = feedback.current_pose.pose.position.x
                            curr_y = feedback.current_pose.pose.position.y
                            
                            if last_check_pos is not None:
                                dist_moved = math.sqrt((curr_x - last_check_pos[0])**2 + (curr_y - last_check_pos[1])**2)
                                
                                if dist_moved < STALL_DISTANCE:
                                    print(f"\n🧱 检测到原地打转/卡死 ({STALL_TIMEOUT}秒仅移动 {dist_moved:.2f}m)！")
                                    print(f"🏃‍♂️ [被动避让] 放弃当前途径点 {index + 1}，前往下一站。")
                                    navigator.cancelTask()
                                    break 
                            
                            last_check_pos = (curr_x, curr_y)
                            last_check_time = current_time

                # --- 报警中断逻辑 (全部点通用) ---
                if patrol_node.alert_received:
                    print("\n🛑【紧急停车】检测到违停！正在处理...")
                    navigator.cancelTask()
                    
                    wait_time = 5
                    for i in range(wait_time, 0, -1):
                        rclpy.spin_once(patrol_node, timeout_sec=0.1) 
                        print(f"⏳ 处理违停中... {i}s (正在更新地图信息)", end='\r')
                        time.sleep(0.9) 
                    
                    print("\n✅ 处理完毕。")
                    patrol_node.alert_received = False
                    
                    print("👀 等待雷达刷新环境 (5s)...")
                    for _ in range(50): 
                        rclpy.spin_once(patrol_node, timeout_sec=0.1)
                        time.sleep(0.1)

                    # 恢复前再次检查（仅限途径点）
                    if not is_task_point:
                        is_blocked, obs_pos = check_if_goal_blocked(goal_pose, patrol_node.known_obstacles, OBSTACLE_RADIUS)
                        if is_blocked:
                            print(f"\n🚫 刚才检测到的违停车辆 {obs_pos} 距离目标太近！")
                            print(f"🏃‍♂️ [动态避让] 放弃当前途径点 {index + 1}，直接前往下一站。")
                            break 

                    print(f"🔄 恢复巡逻，继续前往第 {index + 1} 个站点...")
                    navigator.goToPose(goal_pose)

                    last_check_time = time.time()
                    last_check_pos = None
                    continue

            # --- 结果处理与任务执行 ---
            result = navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                print(f"🎉 到达站点 {index + 1}。")
                
                # ====================================================
                # 到达后的区分处理：执行任务 vs 途径即走
                # ====================================================
                if is_task_point:
                    print(f"🛠️ [任务触发] 开始在站点 {index + 1} 执行特定任务...")
                    # TODO: 替换为你真实的业务代码（播报、拍照、等候）
                    time.sleep(5.0) 
                    print(f"✅ [任务完成] 站点 {index + 1} 任务执行完毕，准备前往下一站！")
                else:
                    time.sleep(1.0) # 途径点稍微停顿
                
            elif result == TaskResult.CANCELED:
                print(f"⚠️ 站点 {index + 1} 导航被取消。")
                
            elif result == TaskResult.FAILED:
                print(f"❌ 站点 {index + 1} 规划失败 (无路可走)。跳过。")
                continue

    except KeyboardInterrupt:
        print("\n\n🛑 收到手动终止信号 (Ctrl+C)！")
        print("⚠️ 正在请求 Nav2 取消任务...")
        navigator.cancelTask()
        print("✅ 任务已取消，程序即将退出。")

    finally:
        print("\n🏁 程序退出。")
        patrol_node.destroy_node()
        rclpy.shutdown()
        exit(0)

if __name__ == '__main__':
    main()