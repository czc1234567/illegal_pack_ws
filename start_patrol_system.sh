#!/bin/bash

# ================= 配置区域 =================
SESSION_NAME="patrol_sys"


# 2. 视觉检测(realsense3d) 所在的工作空间路径
# ⚠️ 请确认这里！因为你的脚本写的是 source install/setup.bash，我假设是在这个目录下
DETECT_WS="/home/agx/chao_ws/illegal_parking_detection/illegal_pack_ws"

# 3. 导航(fishbot) 所在的工作空间路径
# ⚠️ 请确认这里！假设同上，如果不同请修改
NAV_WS="/home/agx/chao_ws/illegal_parking_detection/illegal_pack_ws"

# ===========================================


# -----------------------------------------------------------
# 【Pane 1】: 启动 Realsense Detection (垂直分屏)
# -----------------------------------------------------------
echo "正在启动 视觉检测 节点..."
tmux split-window -v -t "$SESSION_NAME:0.0"
tmux send-keys -t "$SESSION_NAME:0.1" "cd ${DETECT_WS}" C-m
# 环境配置
tmux send-keys -t "$SESSION_NAME:0.1" "source /home/agx/anaconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t "$SESSION_NAME:0.1" "conda activate yolov11" C-m
tmux send-keys -t "$SESSION_NAME:0.1" "source /opt/ros/humble/setup.bash" C-m
# 如果有共用消息包，可以在这里取消注释下一行
# tmux send-keys -t "$SESSION_NAME:0.1" "source ~/chao_ws/robot_ws/install/setup.bash" C-m
tmux send-keys -t "$SESSION_NAME:0.1" "source install/setup.bash" C-m
tmux send-keys -t "$SESSION_NAME:0.1" "export PYTHONPATH=\"\$CONDA_PREFIX/lib/python3.10/site-packages:\$PYTHONPATH\"" C-m
# 启动命令
tmux send-keys -t "$SESSION_NAME:0.1" "ros2 run realsense3d_detection try2_node" C-m


# -----------------------------------------------------------
# 【Pane 2】: 启动 Navigator Patrol (水平分屏)
# -----------------------------------------------------------
echo "正在启动 巡逻导航 节点..."
tmux split-window -h -t "$SESSION_NAME:0.1"
tmux send-keys -t "$SESSION_NAME:0.2" "cd ${NAV_WS}" C-m
# 环境配置
tmux send-keys -t "$SESSION_NAME:0.2" "source /home/agx/anaconda3/etc/profile.d/conda.sh" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "conda activate yolov11" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "source /opt/ros/humble/setup.bash" C-m
# 如果有共用消息包，可以在这里取消注释下一行
# tmux send-keys -t "$SESSION_NAME:0.2" "source ~/chao_ws/robot_ws/install/setup.bash" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "source install/setup.bash" C-m
tmux send-keys -t "$SESSION_NAME:0.2" "export PYTHONPATH=\"\$CONDA_PREFIX/lib/python3.10/site-packages:\$PYTHONPATH\"" C-m
# 启动命令
tmux send-keys -t "$SESSION_NAME:0.2" "ros2 run fishbot_navigator patrol_and_detect" C-m

# -----------------------------------------------------------
# 结束操作
# -----------------------------------------------------------
# 重新排列窗口布局，使其美观
tmux select-layout -t "$SESSION_NAME:0" tiled

echo "所有任务已后台启动！"
echo "使用以下命令查看运行情况："
echo "  tmux attach -t $SESSION_NAME"