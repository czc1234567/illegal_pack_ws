#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class SimpleMapServer(Node):
    def __init__(self):
        super().__init__('simple_map_node') 
        
        # --- 配置部分 ---
        try:
            pkg_share = get_package_share_directory('realsense3d_detection')
            self.yaml_path = os.path.join(pkg_share, 'urdf', 'scans_t12.yaml')
        except Exception as e:
            self.get_logger().error(f"找不到功能包路径: {e}")
            return

        # 设置 QoS: 必须是 Reliable + Transient Local
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        
        # ---------------------------------------------------------
        # 修复点：不要在 __init__ 直接发布，而是创建一个一次性定时器
        # 延迟 1 秒发布，确保连接已建立
        # ---------------------------------------------------------
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.map_msg = None # 用于缓存地图消息

    def timer_callback(self):
        # 如果地图还没加载，就加载一次
        if self.map_msg is None:
            self.map_msg = self.load_map_data()
        
        # 如果加载成功，发布地图
        if self.map_msg:
            self.map_pub.publish(self.map_msg)
            # 对于静态地图，发布一次其实就够了（配合 Transient Local）
            # 但为了保险，也可以设置成长周期发布，或者在这里销毁定时器
            self.get_logger().info("🗺️ 地图已发布 (Timer Event)")
            
            # 如果只想发一次，取消下面这行的注释：
            # self.destroy_timer(self.timer)

    def load_map_data(self):
        """仅负责读取数据并返回 msg，不负责发布"""
        try:
            if not os.path.exists(self.yaml_path):
                self.get_logger().error(f"❌ 找不到YAML文件: {self.yaml_path}")
                return None

            with open(self.yaml_path, 'r') as f:
                map_config = yaml.safe_load(f)
            
            base_dir = os.path.dirname(self.yaml_path)
            img_path = os.path.join(base_dir, map_config['image'])
            
            if not os.path.exists(img_path):
                 self.get_logger().error(f"❌ 找不到图片文件: {img_path}")
                 return None

            image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                self.get_logger().error(f"❌ 无法读取图片: {img_path}")
                return None

            image = cv2.flip(image, 0)

            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.info.resolution = map_config['resolution']
            msg.info.width = image.shape[1]
            msg.info.height = image.shape[0]
            
            origin = map_config['origin'] 
            msg.info.origin = Pose(
                position=Point(x=float(origin[0]), y=float(origin[1]), z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) 
            )

            flat_img = image.flatten()
            map_data = np.full(flat_img.shape, -1, dtype=np.int8)
            
            occ_thresh = map_config.get('occupied_thresh', 0.65) * 255
            free_thresh = map_config.get('free_thresh', 0.25) * 255
            
            # 这里的逻辑是对的：pgm中黑(0)是墙，白(255)是空
            map_data[flat_img <= free_thresh] = 100 
            map_data[flat_img >= occ_thresh] = 0    
            
            msg.data = map_data.tolist()
            return msg
            
        except Exception as e:
            self.get_logger().error(f"加载地图失败: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()