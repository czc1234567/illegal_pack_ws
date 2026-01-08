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
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from collections import defaultdict

class RealSense3DDetectionNode(Node):
    def __init__(self):
        super().__init__('realsense_3d_detection_node')

        # 核心变量
        self.cv_bridge = CvBridge()
        self.color_intrin = None
        self.yolo_model = None
        self.model_loaded = False
        
        # --- 关键优化：数据缓冲区 ---
        self.lock = threading.Lock() # 线程锁，防止读写冲突
        self.latest_color_img = None # 永远只存最新的一帧彩色
        self.latest_depth_img = None # 永远只存最新的一帧深度
        self.new_frame_arrived = False # 标记是否有新帧
        
        # 3D 追踪逻辑变量
        self.sent_target_ids = set()
        self.tracking_buffer = defaultdict(list)
        self.target_classes = [0, 2, 3, 5, 7] # 人, 车, 摩托, 公交, 卡车

        # 1. 启动 YOLO 加载线程
        threading.Thread(target=self.load_yolo_model, daemon=True).start()

        # 2. 启动 图像处理主线程 (这是流畅的关键！)
        # 我们不再依赖 ROS 回调来驱动推理，而是用一个独立的死循环线程
        self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.process_thread.start()

        # 3. ROS 订阅设置
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # 使用近似同步，但回调函数极其轻量，只做赋值
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/camera/depth/image_rect_raw') # 注意：建议使用 aligned_depth_to_color
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=2, slop=0.1)
        self.ts.registerCallback(self.image_sync_callback)

        # 4. 发布器
        self.det_img_pub = self.create_publisher(Image, '/realsense_3d_detection/result_image', 10)
        self.camera_3d_point_pub = self.create_publisher(PointStamped, '/camera_3d_target', 10)

        self.get_logger().info("🚀 极速版启动：采用 生产者-消费者 模型，拒绝卡顿！")

    def load_yolo_model(self):
        try:
            self.yolo_model = YOLO('weight/yolov8n.pt')
            self.model_loaded = True
            self.get_logger().info("✅ YOLOv8 模型加载完毕")
        except Exception as e:
            self.get_logger().error(f"❌ 模型加载失败: {e}")

    def camera_info_callback(self, info_msg):
        if self.color_intrin is None:
            # 简单构建内参对象，方便计算
            import pyrealsense2 as rs # 局部引入
            self.color_intrin = rs.intrinsics()
            self.color_intrin.width, self.color_intrin.height = info_msg.width, info_msg.height
            self.color_intrin.fx, self.color_intrin.fy = info_msg.k[0], info_msg.k[4]
            self.color_intrin.ppx, self.color_intrin.ppy = info_msg.k[2], info_msg.k[5]
            self.color_intrin.model = rs.distortion.brown_conrady
            self.color_intrin.coeffs = list(info_msg.d)

    def image_sync_callback(self, color_msg, depth_msg):
        """
        【生产者】
        这个回调函数必须极快！
        只做一件事：把 ROS 消息转成 OpenCV 格式，扔给 self.latest_... 变量
        然后立刻退出，让 ROS 去接收下一帧。
        """
        try:
            cv_color = self.cv_bridge.imgmsg_to_cv2(color_msg, "bgr8")
            cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            
            with self.lock:
                self.latest_color_img = cv_color
                self.latest_depth_img = cv_depth
                self.latest_header = color_msg.header # 保存时间戳用于发布
                self.new_frame_arrived = True
                
        except Exception as e:
            self.get_logger().error(f"接收转换失败: {e}")

    def processing_loop(self):
        """
        【消费者】
        这个线程独立于 ROS 通信，全速运行 YOLO 和显示。
        """
        import pyrealsense2 as rs # 局部引入
        
        while rclpy.ok():
            if not self.model_loaded or self.color_intrin is None:
                time.sleep(0.1)
                continue

            # 1. 获取最新帧 (加锁读取，极快)
            color_frame = None
            depth_frame = None
            header = None
            
            with self.lock:
                if self.new_frame_arrived:
                    color_frame = self.latest_color_img.copy()
                    depth_frame = self.latest_depth_img.copy()
                    header = self.latest_header
                    self.new_frame_arrived = False # 重置标记
            
            # 如果没有新帧，就稍微睡一下，避免空转烧CPU
            if color_frame is None:
                time.sleep(0.005) 
                continue

            # 2. 开始处理 (耗时操作在这里，完全不影响 ROS 接收数据)
            t_start = time.time()
            
            # YOLO 推理
            results = self.yolo_model.track(
                color_frame, persist=True, conf=0.4, 
                classes=self.target_classes, verbose=False)[0]
            
            if results.boxes and results.boxes.is_track:
                boxes = results.boxes.xyxy.cpu().numpy()
                track_ids = results.boxes.id.int().cpu().numpy()
                cls_ids = results.boxes.cls.int().cpu().numpy()
                names = self.yolo_model.names

                for box, track_id, cls_id in zip(boxes, track_ids, cls_ids):
                    x1, y1, x2, y2 = map(int, box)
                    label = names[cls_id]

                    # 3D 计算
                    dist, coord = self.get_3d_coord(x1, y1, x2, y2, depth_frame)
                    
                    # 5米过滤逻辑
                    if dist <= 0 or dist > 5000:
                        color_res = (0, 0, 255)
                        info_txt = f"{label}-{track_id} >5m"
                    else:
                        color_res = (0, 255, 0)
                        info_txt = f"{label}-{track_id} {dist/1000:.2f}m"
                        # 30帧平滑逻辑
                        self.process_30_frame_logic(track_id, coord, header)

                    # 绘图
                    cv2.rectangle(color_frame, (x1, y1), (x2, y2), color_res, 2)
                    cv2.putText(color_frame, info_txt, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_res, 2)

            # 3. 极速显示 (类似于直连脚本)
            # 计算FPS
            fps = 1.0 / (time.time() - t_start)
            cv2.putText(color_frame, f"FPS: {fps:.1f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            # 拼接显示
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_frame, depth_colormap))
            
            cv2.imshow('RealSense Ultra Fast', images)
            key = cv2.waitKey(1)
            if key == 27 or key == ord('q'):
                rclpy.shutdown()
                break
                
            # (可选) 发布处理后的图片回 ROS，如果不需要可注释掉以节省 CPU
            # self.det_img_pub.publish(self.cv_bridge.cv2_to_imgmsg(color_frame, "bgr8"))

    def get_3d_coord(self, x1, y1, x2, y2, depth_data):
        import pyrealsense2 as rs
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        # 边界检查
        h, w = depth_data.shape
        cx = max(0, min(cx, w-1))
        cy = max(0, min(cy, h-1))
        
        # 取 ROI 中值
        roi = depth_data[max(0, cy-5):min(h, cy+6), max(0, cx-5):min(w, cx+6)]
        valid = roi[roi > 0]
        if len(valid) == 0: return 0, (0,0,0)
        
        dist = np.median(valid)
        p = rs.rs2_deproject_pixel_to_point(self.color_intrin, [cx, cy], dist / 1000.0)
        return dist, p

    def process_30_frame_logic(self, track_id, coord, header):
        if track_id in self.sent_target_ids: return
        
        self.tracking_buffer[track_id].append(coord)
        if len(self.tracking_buffer[track_id]) >= 30:
            arr = np.array(self.tracking_buffer[track_id])
            mx, my, mz = np.median(arr, axis=0)
            
            p = PointStamped()
            p.header = header
            p.header.frame_id = "camera_link"
            p.point.x, p.point.y, p.point.z = mx, my, mz
            self.camera_3d_point_pub.publish(p)
            
            self.get_logger().info(f"🎯 锁定目标 ID:{track_id} 坐标:({mx:.2f}, {my:.2f}, {mz:.2f})")
            self.sent_target_ids.add(track_id)
            del self.tracking_buffer[track_id]

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSense3DDetectionNode()
    try:
        # ROS 的 spin 必须在主线程运行，或者我们把上面的 loop 放在主线程也可以
        # 这里我们让 ROS spin 负责收数据，上面的 thread 负责处理
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()