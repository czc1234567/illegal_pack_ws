import pyrealsense2 as rs
import numpy as np
import cv2
import random
import torch
import time
from ultralytics import YOLO

# 载入 YOLOv8 模型（确保 weight 文件夹存在，且 yolov8n.pt 已下载）
# 若本地无权重，可直接写 model = YOLO('yolov8n.pt')，会自动下载
model = YOLO('weight/yolov8n.pt')


# 计算检测到的物体中心的平均距离 + 对应的三维坐标

def get_mid_pos_and_3dcoord(frame, box, depth_data, intrin, randnum):
    distance_list = []

    # 确定深度索引的中心像素位置 (u, v)
    mid_pixel = [(box[0] + box[2]) // 2, (box[1] + box[3]) // 2]  # (列, 行) 对应 (u, v)
    print("检测框坐标：", box)
    print("物体中心像素坐标(u, v)：", mid_pixel)

    # 确定深度搜索范围
    min_val = min(abs(box[2] - box[0]), abs(box[3] - box[1]))
    print("检测框最小边长：", min_val)

    for i in range(randnum):
        # 添加随机偏差进行深度采样
        bias = random.randint(-min_val // 4, min_val // 4)
        sample_u = int(mid_pixel[0] + bias)
        sample_v = int(mid_pixel[1] + bias)

        # 防止像素坐标超出图像范围（避免索引越界报错）
        if sample_u < 0 or sample_u >= depth_data.shape[1] or sample_v < 0 or sample_v >= depth_data.shape[0]:
            continue

        # 获取索引位置处的深度值（单位：毫米）
        dist = depth_data[sample_v, sample_u]

        # 在帧上可视化采样点
        cv2.circle(frame, (sample_u, sample_v), 4, (255, 0, 0), -1)
        print("采样点深度值(mm)：", dist)

        # 考虑有效的深度值并将其添加到列表中
        if dist > 0:  # 排除无效深度值（0表示无有效深度）
            distance_list.append(dist)

    print('-------------------------')

    # 计算平均深度
    avg_dist = 0
    coord_3d = (0, 0, 0)  # 修正：变量名不以数字开头
    if distance_list:
        # 对深度值进行排序并应用中值滤波
        distance_list = np.array(distance_list)
        distance_list = np.sort(distance_list)[randnum // 2 - randnum // 4:randnum // 2 + randnum // 4]
        avg_dist = np.mean(distance_list)

        # 像素坐标转相机坐标系三维坐标（x,y,z 单位：毫米）
        # rs2_deproject_pixel_to_point(内参, 像素坐标(u,v), 深度值(米))
        temp_coord = rs.rs2_deproject_pixel_to_point(intrin, mid_pixel, avg_dist / 1000.0)
        # 转换为毫米并保留2位小数
        coord_3d = (round(temp_coord[0] * 1000, 2), round(temp_coord[1] * 1000, 2), round(temp_coord[2] * 1000, 2))

    # 返回 平均深度(mm)、中心像素坐标(u,v)、三维坐标(x,y,z mm)
    return avg_dist, mid_pixel, coord_3d


# 在图像上显示检测到的对象，包括类别标签、距离和三维坐标
def dectshow(org_img, boxs, depth_data, intrin):
    img = org_img.copy()

    for box in boxs:
        # 在检测到的物体周围画矩形
        cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)

        # 获取物体的平均深度和三维坐标
        avg_dist, mid_pixel, coord_3d = get_mid_pos_and_3dcoord(org_img, box, depth_data, intrin, 24)
        print("物体平均深度(mm)：", avg_dist)
        print("物体三维坐标(x,y,z mm)：", coord_3d)

        # 显示距离（米，保留2位小数）
        dist_text = f"{(avg_dist / 1000):.2f}m"
        cv2.putText(img, dist_text, (int(box[0]), int(box[1]) - 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 显示三维坐标（毫米，简化显示，避免过长遮挡）
        coord_text = f"X:{coord_3d[0]} Y:{coord_3d[1]} Z:{coord_3d[2]}"
        cv2.putText(img, coord_text, (int(box[0]), int(box[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

    # 显示带注释的图像
    cv2.imshow('detect_3d_result', img)


if __name__ == "__main__":
    # 配置 RealSense 相机的深度和颜色流
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)

    # 尝试启动相机，捕获相机未连接的报错
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"相机启动失败：{e}")
        print("请检查：1. RealSense相机是否已连接 2. 相机驱动是否正常安装")
        exit(1)

    # 关键：创建帧对齐对象，将深度帧对齐到彩色帧（解决像素偏移问题）
    align_to = rs.stream.color
    align = rs.align(align_to)

    try:
        while True:
            # 等待一对连贯的帧：深度和颜色
            frames = pipeline.wait_for_frames()

            # 对齐深度帧到彩色帧
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()  # 对齐后的深度帧
            aligned_color_frame = aligned_frames.get_color_frame()  # 对齐后的彩色帧

            if not aligned_depth_frame or not aligned_color_frame:
                continue

            # 获取相机内参（彩色相机的内参，用于坐标转换）
            color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

            # 将图像转换为 numpy 数组
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            # 对颜色图像使用 YOLOv8 进行目标检测
            results = model(color_image)
            boxes = results[0].boxes.xyxy.cpu().tolist()  # 检测框坐标 [x1,y1,x2,y2]

            # 显示检测结果、距离和三维坐标
            dectshow(color_image, boxes, depth_image, color_intrin)

            # 在深度图像上应用颜色映射
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 水平堆叠彩色和深度图像以进行显示
            images = np.hstack((color_image, depth_colormap))

            # 显示组合图像
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)

            # 等待按键，按 'q' 或 'ESC' 键退出
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    except Exception as e:
        print(f"程序运行出错：{e}")
    finally:
        # 停止流并关闭 RealSense 管道
        pipeline.stop()
        print("相机管道已关闭")
