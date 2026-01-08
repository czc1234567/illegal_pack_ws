#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class CoordTransformer(Node):
    def __init__(self):
        super().__init__('coord_transformer_node')
        # 1. 创建TF2缓冲区和监听器（核心：监听base_link ↔ camera_link变换）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 2. 订阅相机节点发布的camera_link系3D坐标
        self.subscription = self.create_subscription(
            PointStamped,
            '/camera_3d_target',
            self.point_callback,
            10
        )
        self.get_logger().info("✅ TF坐标变换节点已启动，等待相机3D坐标数据...")

    def point_callback(self, msg: PointStamped):
        """回调函数：接收相机坐标 → 转换为base_link坐标 → 打印结果"""
        try:
            # ✅ 核心：查询camera_link到base_link的TF变换关系（超时1秒）
            transform = self.tf_buffer.lookup_transform(
                'base_link',          # 目标坐标系（狗身基坐标系）
                msg.header.frame_id,  # 源坐标系（相机坐标系camera_link）
                rclpy.time.Time()     # 获取最新变换
            )
            
            # ✅ 执行坐标变换：相机坐标 → base_link坐标
            transformed_point = tf2_geometry_msgs.do_transform_point(msg, transform)
            
            # ✅ 终端打印【最终转换好的3D坐标】（核心结果，单位：米）
            self.get_logger().info(
                f"\033[32m✅ 坐标转换成功！base_link坐标系3D坐标：\n"
                f"x = {transformed_point.point.x:.2f} m | "
                f"y = {transformed_point.point.y:.2f} m | "
                f"z = {transformed_point.point.z:.2f} m\033[0m"
            )

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"❌ TF变换失败：{str(e)}")

def main(args=None):
    rclpy.init(args=args)
    coord_transformer = CoordTransformer()
    rclpy.spin(coord_transformer)
    coord_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
