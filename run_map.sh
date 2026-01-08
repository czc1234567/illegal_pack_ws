#!/bin/bash
# 激活conda环境（修正路径）
source /home/agx/anaconda3/etc/profile.d/conda.sh  # 正确路径，无重复的home/czc
conda activate yolov11
# 设置ROS2环境（确保路径正确）
source /opt/ros/humble/setup.bash
# 1. 先source共用消息所在的工作空间（核心：让系统找到消息包）
# source ~/chao_ws/robot_ws/install/setup.bash

#在source当前消息所在空间
source install/setup.bash
# 设置PYTHONPATH（修正conda环境的python路径）
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
# 运行节点
ros2 run realsense3d_detection simple_map_node


