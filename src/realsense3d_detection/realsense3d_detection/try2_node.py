#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('Agg') # 消除 Matplotlib 警告

import rclpy
from rclpy.node import Node
import message_filters
import numpy as np
import cv2
import threading
import time
import json          
import base64        
import requests  
from collections import defaultdict
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
import subprocess

# 消息类型
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool  
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
from cv_bridge import CvBridge
from ultralytics import YOLO

# 尝试导入 TTS
try:
    from custom_msgs.srv import TextToSpeech
except ImportError:
    TextToSpeech = None 

# 尝试导入 Realsense
try:
    import pyrealsense2 as rs
    RS_AVAILABLE = True
except ImportError:
    RS_AVAILABLE = False
    print("⚠️ 警告：未找到 pyrealsense2，3D坐标计算可能不准确！")

class RealSense3DDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_3d_detection_node')
        self.get_logger().info("🚀 [初始化] 正在启动增强型违停检测节点...")

        # 0. 基础配置
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])

        # 1. 区域与目标配置
        self.parking_zones = [
            {'name': 'Zone_A', 'points': [(-12.785, 5.850), (-13.111, 2.153), (-2.456, 1.261), (-2.145, 4.42)]},
            {'name': 'Zone_B', 'points': [(7.534, -49.607), (13.382, -50.300), (19.548, -0.310), (13.111, 0.657)]},
            {'name': 'Zone_C', 'points': [(-45.422, -56.106), (-22.449, -57.463), (-23.201, -65.199), (-45.783, -63.022)]},
            {'name': 'Zone_D', 'points': [(-98.961, -74.009), (-92.856, -74.713), (-92.536, -67.552), (-98.224, -67.135)]},
            {'name': 'Zone_E', 'points': [(53.289, -65.605), (77.914, -67.792), (77.471, -75.729), (52.285, -73.265)]},
            {'name': 'Zone_F', 'points': [(103.042, -61.176), (105.677, -61.559), (111.644, -6.251), (108.801, -6.102)]}
        ]

        self.target_classes = [0, 1, 2, 3, 5, 7] # 加上 0(人) 用于整体测试
        self.coco_mapping = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}
        
        # 2. 状态记录与诊断变量
        self.tts_client = None
        self.MISSION_ID = 1 
        self.violation_start_time = {}      
        self.violation_snapshot_sent = set() 
        self.violation_tts_sent = set()      
        self.permanent_markers_sent = set()  
        self.SNAPSHOT_DELAY = 1.5                    
        self.permanent_marker_array = MarkerArray()

        self.lock = threading.Lock() 
        self.latest_color_img = None 
        self.latest_depth_img = None 
        self.latest_header = None 
        self.new_frame_arrived = False 
        self.model_loaded = False
        self.color_intrin = None
        self.tracking_buffer = defaultdict(list)

        # --- 诊断用统计变量 ---
        self.last_img_time = time.time()
        self.total_frames_received = 0
        self.last_tf_warn_time = 0

        # 3. 初始化工具
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()
        self.api_url = "http://58.209.177.35:8906/api/robot/park-event"

        # 4. 发布者 (统一使用传感器 QoS 避免堵塞)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection/object_markers', 10)
        self.map_marker_pub = self.create_publisher(MarkerArray, '/detection/map_markers', 10)
        self.det_img_pub = self.create_publisher(Image, '/detection/result_image', qos_profile=qos_profile_sensor_data)
        self.alert_pub = self.create_publisher(Bool, '/detection/alert', 10)
        
        qos_perm = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_map_pub = self.create_publisher(MarkerArray, '/detection/result_map', qos_perm)
        self.web_json_pub = self.create_publisher(String, '/detection/web_json', 10)

        # 5. 启动线程与定时器
        self.create_timer(2.0, self.publish_parking_zones) # 提高区域刷新率方便调试
        self.create_timer(3.0, self.diagnostic_watchdog)   # 【新增】看门狗诊断定时器

        threading.Thread(target=self.load_yolo_model, daemon=True).start()
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()

        # 6. 订阅者 (强制使用 qos_profile_sensor_data)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, qos_profile=qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos_profile_sensor_data) 
        
        # 放宽 slop 和 queue_size 解决高负载丢包
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=20, slop=1.5)
        self.ts.registerCallback(self.image_sync_callback)

        self.get_logger().info("✅ [启动完毕] 节点已启动，等待相机和TF树信号...")

    # ==========================
    # 诊断看门狗 (用于排查不报信息的问题)
    # ==========================
    def diagnostic_watchdog(self):
        """每3秒检查一次系统状态，如果卡死会在这里报警"""
        curr_t = time.time()
        
        # 1. 检查 YOLO
        if not self.model_loaded:
            self.get_logger().warn("⏳ [诊断] 等待 YOLO 模型加载...")
            return

        # 2. 检查内参
        if self.color_intrin is None:
            self.get_logger().warn("⚠️ [诊断] 尚未收到相机内参 (/camera_info)。请检查相机驱动是否正常发布！")
            return

        # 3. 检查图像流
        if curr_t - self.last_img_time > 3.0:
            self.get_logger().error(f"❌ [诊断] 已经超过3秒没有收到图像数据！(当前累计收到 {self.total_frames_received} 帧)")
            self.get_logger().warn("👉 请检查：1. realsense 节点是否存活  2. topic 名称对不对  3. 深度图是否开启了 align_depth")

    # ==========================
    # 回调函数
    # ==========================
    def load_yolo_model(self):
        try:
            self.yolo_model = YOLO('weight/yolo11n.pt')
            self.model_loaded = True
            self.get_logger().info("🎯 [模型] YOLO 模型加载成功！")
        except Exception as e:
            self.get_logger().error(f"❌ [模型] YOLO 加载失败，文件路径是否正确？: {e}")

    def camera_info_callback(self, info_msg):
        if self.color_intrin is None and RS_AVAILABLE:
            import pyrealsense2 as rs
            self.color_intrin = rs.intrinsics()
            self.color_intrin.width, self.color_intrin.height = info_msg.width, info_msg.height
            self.color_intrin.fx, self.color_intrin.fy = info_msg.k[0], info_msg.k[4]
            self.color_intrin.ppx, self.color_intrin.ppy = info_msg.k[2], info_msg.k[5]
            self.color_intrin.model = rs.distortion.brown_conrady
            self.color_intrin.coeffs = list(info_msg.d)
            self.get_logger().info("📷 [相机] 成功获取相机内参信息！")

    def image_sync_callback(self, color_msg, depth_msg):
        try:
            if self.total_frames_received == 0:
                self.get_logger().info("🟢 [数据] 成功接收到第一帧图像流！")

            self.total_frames_received += 1
            self.last_img_time = time.time()

            c_img = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            d_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            
            with self.lock: 
                self.latest_color_img = c_img
                self.latest_depth_img = d_img
                self.latest_header = color_msg.header
                self.new_frame_arrived = True
        except Exception as e:
            self.get_logger().error(f"❌ [数据] 图像转换异常: {e}")

    # ==========================
    # 工具函数
    # ==========================
    def create_marker(self, marker_type, marker_id, pose, scale, color, ns="default", frame_id="map", lifetime=0.5, text=""):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = ns, int(marker_id), marker_type, Marker.ADD
        if len(pose) == 3: marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(pose[0]), float(pose[1]), float(pose[2])
        else: marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(pose[0]), float(pose[1]), 0.0
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, scale)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, color)
        marker.lifetime = Duration(seconds=lifetime).to_msg()
        if text: marker.text = text
        return marker

    def is_point_in_polygon(self, x, y, poly_points):
        n = len(poly_points); inside = False; p1x, p1y = poly_points[0]
        for i in range(n + 1):
            p2x, p2y = poly_points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y: xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters: inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def get_3d_coord(self, x1, y1, x2, y2, depth_data):
        if not RS_AVAILABLE or self.color_intrin is None: return 0, (0,0,0)
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        h, w = depth_data.shape
        cx, cy = max(0, min(cx, w-1)), max(0, min(cy, h-1))
        roi = depth_data[max(0, cy-1):min(h, cy+1), max(0, cx-1):min(w, cx+1)]
        valid = roi[roi > 0]
        if len(valid) == 0: return 0, (0,0,0)
        dist = np.median(valid)
        p = rs.rs2_deproject_pixel_to_point(self.color_intrin, [cx, cy], dist / 1000.0)
        return dist, p

    def publish_parking_zones(self):
        marker_array = MarkerArray()
        for i, zone in enumerate(self.parking_zones):
            line = self.create_marker(Marker.LINE_STRIP, i, (0,0,0), (0.3, 0, 0), (0.0, 1.0, 0.0, 1.0), "zones_outline", "map", 0)
            for p in zone['points']:
                pt = Point(); pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.02
                line.points.append(pt)
            line.points.append(line.points[0]) 
            marker_array.markers.append(line)
        self.map_marker_pub.publish(marker_array)

    # ==========================
    # 核心：坐标变换与容错
    # ==========================
    def transform_point_to_map(self, point_stamped):
        try:
            # 尝试查找从 camera 到 map 的转换
            transform = self.tf_buffer.lookup_transform('map', point_stamped.header.frame_id, rclpy.time.Time())
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except Exception as e:
            # 【重要修复】：如果 TF 树断了，每 3 秒报一次错，告诉你错在哪
            curr_t = time.time()
            if curr_t - self.last_tf_warn_time > 3.0:
                self.get_logger().error(f"⛔ [TF树断裂] 无法获取从 '{point_stamped.header.frame_id}' 到 'map' 的坐标系变换！")
                self.get_logger().error(f"详细原因: {e}")
                self.get_logger().warn("👉 请检查：机器人定位/建图程序是否开启？是否存在 base_link 到 camera 的静态变换？")
                self.last_tf_warn_time = curr_t
            return None

    def process_map_logic(self, track_id, coord, header, label, map_markers):
        camera_point = PointStamped()
        camera_point.header = header
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.point.x, camera_point.point.y, camera_point.point.z = map(float, coord)

        map_point = self.transform_point_to_map(camera_point)
        status = 'unknown' 
        new_violation_confirmed = False 

        if map_point is None:
            # TF 转换失败，无法处理地图逻辑
            return status, new_violation_confirmed

        map_x, map_y = map_point.point.x, map_point.point.y
        in_zone = any([self.is_point_in_polygon(map_x, map_y, z['points']) for z in self.parking_zones])
        
        color_green, color_red = (0.0, 1.0, 0.0, 0.8), (1.0, 0.0, 0.0, 0.8)
        footprint = self.create_marker(Marker.CYLINDER, track_id, (map_x, map_y), (0.6, 0.6, 0.05), 
                                     color_green if in_zone else color_red, "footprints", "map", 0.5)
        
        if in_zone:
            status = 'green'
            if track_id in self.violation_start_time: del self.violation_start_time[track_id]
        else:
            status = 'red'
            curr_t = time.time()
            if track_id not in self.violation_start_time: 
                self.violation_start_time[track_id] = curr_t
            
            if (curr_t - self.violation_start_time[track_id]) > self.SNAPSHOT_DELAY:
                if track_id not in self.violation_snapshot_sent:
                    new_violation_confirmed = True
                    self.violation_snapshot_sent.add(track_id)
                    
                    self.alert_pub.publish(Bool(data=True))
                    self.get_logger().warn(f"🚨 [报警] 检测到违停 ID: {track_id}, 坐标: X={map_x:.2f}, Y={map_y:.2f}")

                    if track_id not in self.violation_tts_sent:
                        OUTPUT_FILE = "/home/agx/chao_ws/illegal_parking_detection/illegal_pack_ws/weight/test.mp3"
                        # 【重要修复】：使用 Popen 防止播放语音时堵死整个程序
                        try:
                            subprocess.Popen(["mpg123", "-q", OUTPUT_FILE], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        except Exception as e:
                            self.get_logger().error(f"语音播放失败: {e}")
                        self.violation_tts_sent.add(track_id)

                    if track_id not in self.permanent_markers_sent:
                        perm_m = self.create_marker(Marker.CYLINDER, track_id, (map_x, map_y), 
                                                  (0.6, 0.6, 0.1), (0.8, 0.0, 0.0, 1.0), "permanent_violations", "map", 0)
                        self.permanent_marker_array.markers.append(perm_m)
                        self.permanent_markers_sent.add(track_id)

        map_markers.markers.append(footprint)
        return status, new_violation_confirmed

    # ==========================
    # 主推理循环
    # ==========================
    def processing_loop(self):
        cv2.namedWindow("YOLO Real-time View", cv2.WINDOW_NORMAL)
        last_log_time = time.time()

        while rclpy.ok():
            if not self.model_loaded or self.color_intrin is None: 
                time.sleep(0.5); continue
            
            color_frame, depth_frame, header = None, None, None
            with self.lock:
                if self.new_frame_arrived:
                    color_frame, depth_frame, header = self.latest_color_img.copy(), self.latest_depth_img.copy(), self.latest_header
                    self.new_frame_arrived = False 
            
            if color_frame is None: time.sleep(0.01); continue
            
            # 1. YOLO 目标检测
            results = self.yolo_model.track(color_frame, persist=True, conf=0.4, classes=self.target_classes, verbose=False)[0]
            cam_markers, map_markers = MarkerArray(), MarkerArray()
            
            obj_count = len(results.boxes) if results.boxes else 0
            
            # 定期播报 YOLO 心跳（证明推理线程没死）
            curr_t = time.time()
            if curr_t - last_log_time > 2.0:
                self.get_logger().info(f"👀 [YOLO] 工作正常，当前画面抓取到 {obj_count} 个目标对象")
                last_log_time = curr_t

            if obj_count > 0:
                boxes = results.boxes.xyxy.cpu().numpy()
                cls_ids = results.boxes.cls.int().cpu().numpy()
                has_track = results.boxes.is_track and results.boxes.id is not None
                track_ids = results.boxes.id.int().cpu().numpy() if has_track else [-1]*obj_count

                for box, track_id, cls_id in zip(boxes, track_ids, cls_ids):
                    x1, y1, x2, y2 = map(int, box)
                    label = self.coco_mapping.get(cls_id, f"class_{cls_id}")
                    dist, coord = self.get_3d_coord(x1, y1, x2, y2, depth_frame)
                    
                    status = 'unknown' 
                    
                    if 0 < dist < 8000:
                        disp_id = track_id if track_id != -1 else np.random.randint(1000, 9999)
                        
                        # 只要有坐标，就在 RViz 相机视图下发布 Box Marker (不依赖 TF)
                        cam_box = self.create_marker(Marker.CUBE, disp_id, coord, (0.5, 0.5, 0.5), (0.0, 0.0, 1.0, 0.5), "cam_boxes", "camera_color_optical_frame", 0.5)
                        cam_markers.markers.append(cam_box)
                        
                        # 【关键逻辑】无论有没有 TrackID，都尝试转换为 Map 坐标并处理违停逻辑
                        # 如果没有 TrackID，给一个临时ID进行演示
                        logic_id = track_id if track_id != -1 else disp_id
                        status, _ = self.process_map_logic(logic_id, coord, header, label, map_markers)

                    # 在 OpenCV 画面上画图（不管有没有 TF 都能看到）
                    color = (0, 255, 0) if status == 'green' else ((0, 0, 255) if status == 'red' else (255, 255, 0))
                    cv2.rectangle(color_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(color_frame, f"{label} ID:{track_id} {dist/1000:.1f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 发布给 RViz
            self.marker_pub.publish(cam_markers)      
            self.map_marker_pub.publish(map_markers)
            if len(self.permanent_marker_array.markers) > 0: 
                self.result_map_pub.publish(self.permanent_marker_array)

            # 把带有绘制框的图像发送给 Web 端/RViz
            try:
                img_msg = self.cv_bridge.cv2_to_imgmsg(color_frame, "bgr8")
                img_msg.header = header
                self.det_img_pub.publish(img_msg)
            except: pass

            # 2. 本地 OpenCV 可视化 (按 q 退出)
            try:
                cv2.imshow("YOLO Real-time View", color_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): 
                    break
            except Exception as e:
                self.get_logger().warn(f"🖥️ 无法显示可视化窗口 (如果你在SSH无桌面环境下，可忽略此警告): {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSense3DDetectionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()