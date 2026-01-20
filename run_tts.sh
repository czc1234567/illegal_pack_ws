#!/bin/bash
# 跨工作空间启动tts_node

# 1. 激活conda环境
source /home/agx/anaconda3/etc/profile.d/conda.sh
conda activate yolov11

# 2. 配置ROS2环境（系统级 + 工作空间级）
source /opt/ros/humble/setup.bash
# source ~/chao_ws/robot_ws/install/setup.bash  # 有共用消息包则取消注释
source /home/agx/rayna/robotplatform-main/install/setup.bash

# 3. 设置Python路径
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"

# 4. 启动tts节点
ros2 run go2_tts tts_node

# 退出时取消conda激活
conda deactivate