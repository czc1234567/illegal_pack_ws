#!/bin/bash

# 定义 tmux 会话名称
SESSION_NAME="ros2_yolo"

# 1. 检查并清理旧会话
tmux has-session -t $SESSION_NAME 2>/dev/null
if [ $? == 0 ]; then
  echo "正在清理旧会话..."
  tmux kill-session -t $SESSION_NAME
fi

echo "正在启动全功能集成会话 (含相机启动)..."

# 2. 创建新会话
tmux new-session -d -s $SESSION_NAME

# 3. 设置四分屏布局
# 先左右对半切 (左 0.0, 右 0.1)
tmux split-window -h -t $SESSION_NAME
# 将右侧窗口切成三段 (右上 0.1, 右中 0.2, 右下 0.3)
tmux split-window -v -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME:0.1

# 4. 定义环境初始化命令
# 包含 Conda, ROS2 Humble 以及你的工作空间 setup
SETUP_ENV="source /home/agx/anaconda3/etc/profile.d/conda.sh && conda activate yolov11 && source /opt/ros/humble/setup.bash && source /home/agx/chao_ws/illegal_parking_detection/illegal_pack_ws/install/setup.bash && export PYTHONPATH=\"\$CONDA_PREFIX/lib/python3.10/site-packages:\$PYTHONPATH\""

# --- 开始向各窗口发送指令 ---

# Pane 0: 巡检导航 (左侧大窗)
tmux send-keys -t $SESSION_NAME:0.0 "$SETUP_ENV" C-m
tmux send-keys -t $SESSION_NAME:0.0 "echo '>>> 启动巡检导航...'; taskset -c 0-2 ros2 run fishbot_navigator patrol_and_detect" C-m

# Pane 1: YOLO 检测节点 (右上)
tmux send-keys -t $SESSION_NAME:0.1 "$SETUP_ENV" C-m
tmux send-keys -t $SESSION_NAME:0.1 "echo '>>> 启动 YOLO 检测逻辑...'; taskset -c 0-2 ros2 run realsense3d_detection detection_node" C-m

# Pane 2: RealSense 相机启动 (右中) 【新增】
# 按照要求：cd 到 agent_ws 并执行启动指令
tmux send-keys -t $SESSION_NAME:0.2 "$SETUP_ENV" C-m
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/agent_ws/ && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo '>>> 正在启动 RealSense 相机 (对齐模式)...'; ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true" C-m

# Pane 3: 静态 TF 发布 (右下)
tmux send-keys -t $SESSION_NAME:0.3 "$SETUP_ENV" C-m
tmux send-keys -t $SESSION_NAME:0.3 "echo '>>> 发布静态坐标变换 TF...';" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0.0 --z 0.0 --roll -1.5708 --pitch 0.0 --yaw -1.5708 --frame-id base_link --child-frame-id camera_color_optical_frame" C-m

echo "------------------------------------------------"
echo "✅ 全部节点已启动！"
echo "布局说明："
echo "  [ 左 ] : 巡检导航"
echo "  [右上] : YOLO检测"
echo "  [右中] : 相机驱动 (agent_ws)"
echo "  [右下] : 静态 TF"
echo "------------------------------------------------"

# 自动进入 tmux
tmux attach -t $SESSION_NAME