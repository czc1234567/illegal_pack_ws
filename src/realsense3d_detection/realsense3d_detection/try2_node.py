#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import message_filters
import numpy as np
import cv2
import threading
import time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

# --- TF 相关库 ---
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
# ----------------

from cv_bridge import CvBridge
from ultralytics import YOLO
from collections import defaultdict

class RealSense3DDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_3d_detection_node')

        # ==========================================
        # 1. 区域配置 (请确保坐标是相对于 map 的)
        # ==========================================
        self.parking_zones = [
            {'name': 'Zone_A', 'x_min': -1.0, 'x_max': 1.0, 'y_min': -2.0, 'y_max': 0.0},
            {'name': 'Zone_B', 'x_min': 2.5, 'x_max': 4.5, 'y_min': -1.0,  'y_max': 1.0}
        ]

        # ==========================================
        # 2. 检测类别 (人、自行车、车、摩托、大巴、卡车)
        # ==========================================
        # 任何在这里列出的类别，都会被检测并判断是否在区域内
        self.target_classes = [0, 1, 2, 3, 5, 7] 

        # --- 初始化 TF ---
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
        
        self.sent_target_ids = set()
        self.tracking_buffer = defaultdict(list)

        # 发布器
        self.marker_pub = self.create_publisher(MarkerArray, '/detection/object_markers', 10) # 空中3D框
        self.map_marker_pub = self.create_publisher(MarkerArray, '/detection/map_markers', 10) # 地面区域+脚印
        self.det_img_pub = self.create_publisher(Image, '/detection/result_image', 10)

        # 定时发布区域显示
        self.create_timer(1.0, self.publish_parking_zones)

        threading.Thread(target=self.load_yolo_model, daemon=True).start()
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()

        # 订阅器
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw') 
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=2, slop=0.1)
        self.ts.registerCallback(self.image_sync_callback)

        self.get_logger().info("🚀 启动：全类别区域监控模式")

    # =========================================
    # 在地图上画出半透明的绿色区域
    # =========================================
    def publish_parking_zones(self):
        marker_array = MarkerArray()
        
        for i, zone in enumerate(self.parking_zones):
            # 实心方块
            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = self.get_clock().now().to_msg()
            cube.ns = "zones_fill"
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            
            width = zone['x_max'] - zone['x_min']
            height = zone['y_max'] - zone['y_min']
            cx = (zone['x_min'] + zone['x_max']) / 2.0
            cy = (zone['y_min'] + zone['y_max']) / 2.0
            
            cube.pose.position.x = cx
            cube.pose.position.y = cy
            cube.pose.position.z = 0.02 
            
            cube.scale.x = abs(width)
            cube.scale.y = abs(height)
            cube.scale.z = 0.05
            
            # 区域本身显示为半透明绿色，表示这是“安全区”
            cube.color.r = 0.0; cube.color.g = 1.0; cube.color.b = 0.0; cube.color.a = 0.3
            
            # 文字
            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = cube.header.stamp
            text.ns = "zones_text"
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = cx
            text.pose.position.y = cy
            text.pose.position.z = 0.5
            text.scale.z = 0.4
            text.color.a = 1.0; text.color.r = 1.0; text.color.g = 1.0; text.color.b = 1.0
            text.text = zone['name']

            marker_array.markers.append(cube)
            marker_array.markers.append(text)
        
        self.map_marker_pub.publish(marker_array)

    def load_yolo_model(self):
        try:
            self.yolo_model = YOLO('weight/yolo11n.engine')
            self.model_loaded = True
            self.get_logger().info("✅ 模型加载完毕")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")

    def camera_info_callback(self, info_msg):
        if self.color_intrin is None:
            import pyrealsense2 as rs 
            self.color_intrin = rs.intrinsics()
            self.color_intrin.width = info_msg.width
            self.color_intrin.height = info_msg.height
            self.color_intrin.fx, self.color_intrin.fy = info_msg.k[0], info_msg.k[4]
            self.color_intrin.ppx, self.color_intrin.ppy = info_msg.k[2], info_msg.k[5]
            self.color_intrin.model = rs.distortion.brown_conrady
            self.color_intrin.coeffs = list(info_msg.d)

    def image_sync_callback(self, color_msg, depth_msg):
        try:
            cv_color = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            with self.lock:
                self.latest_color_img = cv_color
                self.latest_depth_img = cv_depth
                self.latest_header = color_msg.header 
                self.new_frame_arrived = True
        except Exception as e:
            self.get_logger().error(f"接收转换失败: {e}")

    def processing_loop(self):
        while rclpy.ok():
            if not self.model_loaded or self.color_intrin is None:
                time.sleep(0.1); continue

            color_frame = None; depth_frame = None; header = None
            with self.lock:
                if self.new_frame_arrived:
                    color_frame = self.latest_color_img.copy()
                    depth_frame = self.latest_depth_img.copy()
                    header = self.latest_header
                    self.new_frame_arrived = False 
            
            if color_frame is None: time.sleep(0.005); continue
            
            # YOLO Track
            results = self.yolo_model.track(
                color_frame, persist=True, conf=0.4, 
                classes=self.target_classes, verbose=False)[0]
            
            cam_markers = MarkerArray()
            map_markers = MarkerArray()

            if results.boxes and results.boxes.is_track:
                boxes = results.boxes.xyxy.cpu().numpy()
                track_ids = results.boxes.id.int().cpu().numpy()
                cls_ids = results.boxes.cls.int().cpu().numpy()
                names = self.yolo_model.names

                for box, track_id, cls_id in zip(boxes, track_ids, cls_ids):
                    x1, y1, x2, y2 = map(int, box)
                    label = names[cls_id]

                    dist, coord = self.get_3d_coord(x1, y1, x2, y2, depth_frame)
                    
                    if dist > 0 and dist < 8000:
                        # 1. 空中 3D 框 (保持原样，只负责显示物体在哪里)
                        self.add_cam_marker(cam_markers, track_id, label, coord, header)

                        # 2. 地图逻辑 (核心修改：统一所有类别的判断)
                        status_color = self.process_map_logic(track_id, coord, header, label, map_markers)
                        
                        # 画面上的文字颜色也跟随状态 (绿=正常，红=区域外)
                        text_color = (0, 255, 0) if status_color == 'green' else (0, 0, 255)
                        cv2.rectangle(color_frame, (x1, y1), (x2, y2), text_color, 2)
                        cv2.putText(color_frame, f"{label} {dist/1000:.1f}m", (x1, y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)

            self.marker_pub.publish(cam_markers)      
            self.map_marker_pub.publish(map_markers)  
            
            cv2.imshow('Detection', color_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown(); break

    def get_3d_coord(self, x1, y1, x2, y2, depth_data):
        import pyrealsense2 as rs
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        h, w = depth_data.shape
        cx = max(0, min(cx, w-1)); cy = max(0, min(cy, h-1))
        
        roi = depth_data[max(0, cy-5):min(h, cy+6), max(0, cx-5):min(w, cx+6)]
        valid = roi[roi > 0]
        if len(valid) == 0: return 0, (0,0,0)
        
        dist = np.median(valid)
        p = rs.rs2_deproject_pixel_to_point(self.color_intrin, [cx, cy], dist / 1000.0)
        return dist, p

    # =========================================================
    # 核心逻辑修改：不分人/车，只分“在区域内”或“在区域外”
    # =========================================================
    def process_map_logic(self, track_id, coord, header, label, map_markers):
        camera_point = PointStamped()
        camera_point.header = header
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.point.x, camera_point.point.y, camera_point.point.z = float(coord[0]), float(coord[1]), float(coord[2])

        # 尝试转换坐标
        map_point = self.transform_point_to_map(camera_point)
        
        status = 'unknown' # 默认状态

        if map_point:
            map_x = map_point.point.x
            map_y = map_point.point.y

            # 1. 判断是否在任何一个区域内
            in_zone = False
            zone_name = "None"
            for zone in self.parking_zones:
                if (zone['x_min'] <= map_x <= zone['x_max'] and 
                    zone['y_min'] <= map_y <= zone['y_max']):
                    in_zone = True
                    zone_name = zone['name']
                    break
            
            # 2. 设置脚印 Marker
            footprint = Marker()
            footprint.header.frame_id = "map"
            footprint.header.stamp = self.get_clock().now().to_msg()
            footprint.ns = "footprints"
            footprint.id = int(track_id)
            footprint.type = Marker.CYLINDER 
            footprint.action = Marker.ADD
            footprint.pose.position.x = map_x
            footprint.pose.position.y = map_y
            footprint.pose.position.z = 0.0 
            footprint.scale.x = 0.6; footprint.scale.y = 0.6; footprint.scale.z = 0.05
            footprint.color.a = 0.8

            # --- 统一判断逻辑 ---
            if in_zone:
                # 在区域内 -> 绿色 (安全/合规)
                footprint.color.r = 0.0; footprint.color.g = 1.0; footprint.color.b = 0.0
                status = 'green'
            else:
                # 在区域外 -> 红色 (警告/违规)
                footprint.color.r = 1.0; footprint.color.g = 0.0; footprint.color.b = 0.0
                status = 'red'
            
            footprint.lifetime = Duration(seconds=0.2).to_msg() 
            map_markers.markers.append(footprint)

            # 3. 30帧稳定输出日志
            self.tracking_buffer[track_id].append(map_point)
            if len(self.tracking_buffer[track_id]) >= 30:
                # 只有当这个ID之前没报过警，才打印
                if track_id not in self.sent_target_ids:
                    obj_name = f"{label.upper()}-{track_id}"
                    pos_str = f"({map_x:.1f}, {map_y:.1f})"
                    
                    if in_zone:
                         self.get_logger().info(f"✅ [合规] {obj_name} 位于区域 {zone_name} 内 {pos_str}")
                    else:
                         self.get_logger().warn(f"🚨 [越界] {obj_name} 位于区域外! {pos_str}")
                    
                    self.sent_target_ids.add(track_id)
                
                # 清理缓存，防止无限增长
                del self.tracking_buffer[track_id]
        
        return status

    def transform_point_to_map(self, point_stamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', point_stamped.header.frame_id, rclpy.time.Time())
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except Exception:
            return None

    def add_cam_marker(self, marker_array, track_id, label, coord, header):
        box = Marker()
        box.header = header 
        box.header.frame_id = "camera_color_optical_frame"
        box.ns = "cam_boxes"
        box.id = int(track_id)
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x, box.pose.position.y, box.pose.position.z = float(coord[0]), float(coord[1]), float(coord[2])
        box.scale.x, box.scale.y, box.scale.z = 0.5, 0.5, 0.5
        box.color.a = 0.5; box.color.b = 1.0
        box.lifetime = Duration(seconds=0.2).to_msg()
        marker_array.markers.append(box)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSense3DDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()