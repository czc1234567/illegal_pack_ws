#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import message_filters
import numpy as np
import cv2
import threading
import time
import json          
import base64        
import requests  # 【新增】引入 requests 库用于发送 HTTP POST 请求
from collections import defaultdict
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.duration import Duration
import subprocess

# 消息类型
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool  # 引入 Bool 类型
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
from cv_bridge import CvBridge
from ultralytics import YOLO

# 导入 TTS 服务消息
try:
    from custom_msgs.srv import TextToSpeech
except ImportError:
    print("❌ 无法导入 custom_msgs.srv.TextToSpeech，请检查 source 顺序！")
    TextToSpeech = None 

class RealSense3DDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_3d_detection_node')

        # 0. 基础配置
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)])

        # 【新增】接口配置
        self.api_url = "http://58.209.177.35:8906/api/robot/park-event"

        # 1. 区域配置
        self.parking_zones = [
            {'name': 'Zone_A', 'points': [(-12.785, 5.850), (-13.111, 2.153), (-2.456, 1.261), (-2.145, 4.42)]},
            {'name': 'Zone_B', 'points': [(7.534, -49.607), (13.382, -50.300), (19.548, -0.310), (13.111, 0.657)]},
            {'name': 'Zone_C', 'points': [(-45.422, -56.106), (-22.449, -57.463), (-23.201, -65.199), (-45.783, -63.022)]},
            {'name': 'Zone_D', 'points': [(-98.961, -74.009), (-92.856, -74.713), (-92.536, -67.552), (-98.224, -67.135)]},
            {'name': 'Zone_E', 'points': [(53.289, -65.605), (77.914, -67.792), (77.471, -75.729), (52.285, -73.265)]},
            {'name': 'Zone_F', 'points': [(103.042, -61.176), (105.677, -61.559), (111.644, -6.251), (108.801, -6.102)]},
            {'name': 'Zone_G', 'points': [(-83.5996, -41.1149),(-74.4677, -41.9531),(-74.6921, -47.8101),(-84.0977, -46.8601)]}
        ]

        self.target_classes = [1, 2, 3, 5, 7] 
        self.coco_mapping = { 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}

        # 2. TTS 与 状态记录配置
        self.tts_client = None
        self.MISSION_ID = 1 
        self.tts_alert_text = "此处不是停车区域，请尽快驶离"
        
        self.violation_start_time = {}      
        self.violation_snapshot_sent = set() 
        self.violation_tts_sent = set()      
        self.permanent_markers_sent = set()  
        self.SNAPSHOT_DELAY = 1.5                    
        
        self.permanent_marker_array = MarkerArray()
        
        if TextToSpeech:
            self.tts_client = self.create_client(TextToSpeech, '/tts')
            threading.Thread(target=self.check_tts_service, daemon=True).start()
        
        # 3. 初始化工具
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cv_bridge = CvBridge()
        self.color_intrin = None
        self.yolo_model = None
        self.model_loaded = False
        self.lock = threading.Lock() 
        self.latest_color_img = None 
        self.latest_depth_img = None 
        self.latest_header = None 
        self.new_frame_arrived = False 
        self.tracking_buffer = defaultdict(list)

        # 4. 发布者
        self.marker_pub = self.create_publisher(MarkerArray, '/detection/object_markers', 10)
        self.map_marker_pub = self.create_publisher(MarkerArray, '/detection/map_markers', 10)
        self.det_img_pub = self.create_publisher(Image, '/detection/result_image', 10)
        self.alert_pub = self.create_publisher(Bool, '/detection/alert', 10)

        # 永久标记发布者
        qos_permanent = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.result_map_pub = self.create_publisher(MarkerArray, '/detection/result_map', qos_permanent)

        # Web JSON 发布者
        self.web_json_pub = self.create_publisher(String, '/detection/web_json', 10)

        self.create_timer(1.0, self.publish_parking_zones)

        threading.Thread(target=self.load_yolo_model, daemon=True).start()
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()

        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw') 
        self.ts = message_filters.ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=2, slop=0.5)
        self.ts.registerCallback(self.image_sync_callback)

        self.get_logger().info("🚀 系统启动：检测 + TTS + 永久标记 + WebJSON/HTTP推送 + 导航联动")

    # ==========================
    # 通用 Marker 创建函数
    # ==========================
    def create_marker(self, marker_type, marker_id, pose, scale, color, ns="default", frame_id="map", lifetime=0.2, text=""):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = int(marker_id)
        marker.type = marker_type
        marker.action = Marker.ADD
        
        if len(pose) == 3:
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(pose[0]), float(pose[1]), float(pose[2])
        else:
            marker.pose.position.x, marker.pose.position.y = float(pose[0]), float(pose[1])
            marker.pose.position.z = 0.0

        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        
        if lifetime == 0:
            marker.lifetime = Duration(seconds=0).to_msg() 
        else:
            marker.lifetime = Duration(seconds=lifetime).to_msg()
            
        if text: marker.text = text
        return marker

    # ==========================
    # 区域检测逻辑抽离
    # ==========================
    def check_position_in_zones(self, x, y):
        """返回是否在区域内，以及区域名称"""
        for zone in self.parking_zones:
            if self.is_point_in_polygon(x, y, zone['points']):
                return True, zone['name']
        return False, None

    def check_tts_service(self):
        if self.tts_client and not self.tts_client.wait_for_service(timeout_sec=5.0): pass
        else: self.get_logger().info("✅ TTS服务已连接")

    def send_tts_request(self, text):
        if self.tts_client and self.tts_client.service_is_ready():
            req = TextToSpeech.Request(); req.mission = self.MISSION_ID; req.text = text; self.tts_client.call_async(req)

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

    def load_yolo_model(self):
        self.yolo_model = YOLO('weight/yolo11n.pt'); self.model_loaded = True

    def camera_info_callback(self, info_msg):
        if self.color_intrin is None:
            import pyrealsense2 as rs; self.color_intrin = rs.intrinsics()
            self.color_intrin.width = info_msg.width; self.color_intrin.height = info_msg.height
            self.color_intrin.fx, self.color_intrin.fy = info_msg.k[0], info_msg.k[4]
            self.color_intrin.ppx, self.color_intrin.ppy = info_msg.k[2], info_msg.k[5]
            self.color_intrin.model = rs.distortion.brown_conrady; self.color_intrin.coeffs = list(info_msg.d)

    def image_sync_callback(self, color_msg, depth_msg):
        try:
            self.latest_color_img = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            self.latest_depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            with self.lock: self.latest_header = color_msg.header; self.new_frame_arrived = True
        except: pass

    def get_3d_coord(self, x1, y1, x2, y2, depth_data):
        import pyrealsense2 as rs
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        h, w = depth_data.shape
        cx = max(0, min(cx, w-1)); cy = max(0, min(cy, h-1))
        roi = depth_data[max(0, cy-1):min(h, cy+1), max(0, cx-1):min(w, cx+1)]
        valid = roi[roi > 0]
        if len(valid) == 0: return 0, (0,0,0)
        dist = np.median(valid)
        p = rs.rs2_deproject_pixel_to_point(self.color_intrin, [cx, cy], dist / 1000.0)
        return dist, p

    # ==========================
    # 区域发布
    # ==========================
    def publish_parking_zones(self):
        marker_array = MarkerArray()
        for i, zone in enumerate(self.parking_zones):
            points = zone['points']
            if len(points) < 3: continue
            
            line = self.create_marker(Marker.LINE_STRIP, i, (0,0,0), (0.3, 0, 0), (0.0, 1.0, 0.0, 1.0), "zones_outline", "map", 0)
            for p in points:
                pt = Point(); pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.02
                line.points.append(pt)
            line.points.append(line.points[0]) # 闭合
            marker_array.markers.append(line)

            cx = sum([p[0] for p in points]) / len(points)
            cy = sum([p[1] for p in points]) / len(points)
            text = self.create_marker(Marker.TEXT_VIEW_FACING, i+100, (cx, cy, 0.5), (0, 0, 0.4), (1.0, 1.0, 1.0, 1.0), "zones_text", "map", 0, zone['name'])
            marker_array.markers.append(text)
        
        self.map_marker_pub.publish(marker_array)

    # ==========================
    # JSON 发布与 HTTP 推送 (核心修改区)
    # ==========================
    def publish_web_json(self, cv_image, marker_array, header):
        try:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            _, buffer = cv2.imencode('.jpg', cv_image, encode_param)
            img_b64 = base64.b64encode(buffer).decode('utf-8')
            
            markers_data = [
                {'id': m.id, 'x': round(m.pose.position.x, 3), 'y': round(m.pose.position.y, 3)}
                for m in marker_array.markers
            ]
            
            # 标准 JSON 对象 Payload
            payload = {
                'timestamp': header.stamp.sec,
                'frame_id': header.frame_id,
                'image_base64': img_b64,
                'markers': markers_data
            }
            
            # 1. 保留原本的 ROS Topic 发布
            msg = String()
            msg.data = json.dumps(payload)
            self.web_json_pub.publish(msg)
            self.get_logger().info(f"📤 Web JSON 话题推送: {len(markers_data)} 个标记")

            # 2. 【新增】HTTP API 推送功能 (使用多线程以防阻塞)
            def send_http_request():
                try:
                    headers = {'Content-Type': 'application/json'}
                    # 直接将组装好的 payload 作为 JSON 发送
                    response = requests.post(self.api_url, json=payload, headers=headers, timeout=5)
                    
                    if response.status_code == 200:
                        self.get_logger().info("✅ 成功推送数据至远程 HTTP 接口")
                    else:
                        self.get_logger().warning(f"⚠️ HTTP 接口推送失败, 状态码: {response.status_code}")
                except Exception as req_e:
                    self.get_logger().error(f"❌ HTTP 请求异常: {req_e}")

            # 启动线程发送 HTTP 请求
            threading.Thread(target=send_http_request, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"JSON组装/发布失败: {e}")

    def publish_snapshot(self, cv_image, header):
        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            img_msg.header = header
            self.det_img_pub.publish(img_msg)
        except Exception as e: pass

    def transform_point_to_map(self, point_stamped):
        try:
            transform = self.tf_buffer.lookup_transform('map', point_stamped.header.frame_id, rclpy.time.Time())
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except: return None

    # ==========================
    # 业务逻辑 (包含发送停车信号)
    # ==========================
    def process_map_logic(self, track_id, coord, header, label, map_markers):
        camera_point = PointStamped()
        camera_point.header = header
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.point.x, camera_point.point.y, camera_point.point.z = map(float, coord)

        map_point = self.transform_point_to_map(camera_point)
        status = 'unknown' 
        new_violation_confirmed = False 

        if map_point:
            map_x = map_point.point.x
            map_y = map_point.point.y
            
            in_zone, _ = self.check_position_in_zones(map_x, map_y)
            
            color_green = (0.0, 1.0, 0.0, 0.8)
            color_red = (1.0, 0.0, 0.0, 0.8)
            
            footprint = self.create_marker(Marker.CYLINDER, track_id, (map_x, map_y), (0.6, 0.6, 0.05), 
                                         color_green if in_zone else color_red, "footprints", "map", 0.2)
            
            if in_zone:
                status = 'green'
                if track_id in self.violation_start_time: del self.violation_start_time[track_id]
                if track_id in self.violation_snapshot_sent: self.violation_snapshot_sent.remove(track_id)
            else:
                status = 'red'
                curr_t = time.time()
                if track_id not in self.violation_start_time: self.violation_start_time[track_id] = curr_t
                
                # 满2秒逻辑，确认违停
                if (curr_t - self.violation_start_time[track_id]) > self.SNAPSHOT_DELAY:
                    # A. 抓拍标记 & 发送停车信号
                    if track_id not in self.violation_snapshot_sent:
                        new_violation_confirmed = True
                        self.violation_snapshot_sent.add(track_id)
                        
                        msg = Bool()
                        msg.data = True
                        self.alert_pub.publish(msg)
                        self.get_logger().warn(f"🚨 发送停车信号！检测到 ID: {track_id}")

                    # B. 语音 (使用 mpg123 后台纯净播放)
                    # B. 语音 (强行直通底层 ALSA 驱动，解决后台队列阻塞问题)
                    if track_id not in self.violation_tts_sent:
                        OUTPUT_FILE = "/home/agx/chao_ws/illegal_parking_detection/illegal_pack_ws/weight/test.mp3"

                        # 核心修改：加入 "-o", "alsa" 强行指定走底层驱动，不走上层容易休眠的服务
                        mpg123_cmd = ["mpg123", "-o", "alsa", "-q", OUTPUT_FILE]
                        
                        try:
                            # 启动播放进程
                            subprocess.Popen(mpg123_cmd, 
                                             stdin=subprocess.DEVNULL, 
                                             stdout=subprocess.DEVNULL, 
                                             stderr=subprocess.DEVNULL)
                                             
                            self.get_logger().info(f"🔊 已触发违停语音 (ID: {track_id})")
                                
                        except Exception as e:
                            self.get_logger().error(f"语音播放出错: {e}")

                        self.violation_tts_sent.add(track_id)

                    # C. 永久标记
                    if track_id not in self.permanent_markers_sent:
                        perm_m = self.create_marker(Marker.CYLINDER, track_id, (map_x, map_y), 
                                                  (0.6, 0.6, 0.1), (0.8, 0.0, 0.0, 1.0), "permanent_violations", "map", 0)
                        self.permanent_marker_array.markers.append(perm_m)
                        self.permanent_markers_sent.add(track_id)

            map_markers.markers.append(footprint)
            self.tracking_buffer[track_id].append(map_point)
            if len(self.tracking_buffer[track_id]) >= 30: del self.tracking_buffer[track_id]

        return status, new_violation_confirmed

    def processing_loop(self):
        while rclpy.ok():
            if not self.model_loaded or self.color_intrin is None: time.sleep(0.1); continue
            color_frame = None; depth_frame = None; header = None
            with self.lock:
                if self.new_frame_arrived:
                    color_frame = self.latest_color_img.copy(); depth_frame = self.latest_depth_img.copy(); header = self.latest_header; self.new_frame_arrived = False 
            if color_frame is None: time.sleep(0.005); continue
            
            results = self.yolo_model.track(color_frame, persist=True, conf=0.4, classes=self.target_classes, verbose=False)[0]
            cam_markers = MarkerArray(); map_markers = MarkerArray()
            trigger_json = False

            if results.boxes and results.boxes.is_track:
                for box, track_id, cls_id in zip(results.boxes.xyxy.cpu().numpy(), results.boxes.id.int().cpu().numpy(), results.boxes.cls.int().cpu().numpy()):
                    x1, y1, x2, y2 = map(int, box)
                    label = self.coco_mapping.get(cls_id, f"class_{cls_id}")
                    dist, coord = self.get_3d_coord(x1, y1, x2, y2, depth_frame)
                    
                    if dist > 0 and dist < 8000:
                        cam_box = self.create_marker(Marker.CUBE, track_id, coord, (0.5, 0.5, 0.5), (0.0, 0.0, 1.0, 0.5), "cam_boxes", "camera_color_optical_frame", 0.2)
                        cam_markers.markers.append(cam_box)
                        
                        status, new_viol = self.process_map_logic(track_id, coord, header, label, map_markers)
                        if new_viol: trigger_json = True

                        cv2.rectangle(color_frame, (x1, y1), (x2, y2), (0, 255, 0) if status == 'green' else (0, 0, 255), 2)
                        cv2.putText(color_frame, f"{label} {dist/1000:.1f}m", (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if status == 'green' else (0, 0, 255), 2)
            
            self.marker_pub.publish(cam_markers)      
            self.map_marker_pub.publish(map_markers)
            if len(self.permanent_marker_array.markers) > 0: self.result_map_pub.publish(self.permanent_marker_array)

            if trigger_json:
                self.publish_snapshot(color_frame, header)
                self.publish_web_json(color_frame, self.permanent_marker_array, header)

    def destroy_node(self):
        cv2.destroyAllWindows(); super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = RealSense3DDetectionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()