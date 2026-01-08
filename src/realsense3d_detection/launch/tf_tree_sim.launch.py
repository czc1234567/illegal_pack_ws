# 文件路径：~/ssd/ks_work/Illegal_parking_detection/illegal_pack_ws/src/realsense3d_detection/launch/tf_tree_sim.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ✅ 核心修正：替换为你的真实包名（已改好，无需再动）
    pkg_name = "realsense3d_detection"
    # 获取功能包安装后的绝对路径（ROS2标准写法）
    pkg_share_path = get_package_share_directory(pkg_name)
    
    # ✅ 拼接URDF文件路径（自动适配编译后的路径，永不报错）
    urdf_file_full_path = os.path.join(pkg_share_path, "urdf", "go2_camera.urdf")

    # 启动robot_state_publisher节点，广播TF树
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_file_full_path],  # 传入URDF绝对路径
        parameters=[{"use_sim_time": False}]  # 单机测试关闭仿真时间
    )

    # 返回启动描述（ROS2固定格式）
    return LaunchDescription([robot_state_pub_node])
