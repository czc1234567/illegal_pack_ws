import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'realsense3d_detection'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 设置 RViz 配置文件路径
    # 假设你把配置保存到了包内的 rviz 文件夹下的 default.rviz
    # 如果该文件不存在，RViz 依然会启动，但会是默认空视图
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    return LaunchDescription([
        # --- 节点 1: 地图服务器 ---
        Node(
            package=pkg_name,
            executable='simple_map_node',
            name='map_server',
            output='screen'
        ),

        # --- 节点 2: 静态坐标变换 (数据来自你的 tf2_echo) ---
        # 对应关系: map -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_camera',
            arguments=[
                '--x', '-8.667',
                '--y', '6.105',
                '--z', '0.000',
                '--yaw', '2.069',    # 对应 RPY 中的第 3 个数
                '--pitch', '0.000',  # 对应 RPY 中的第 2 个数
                '--roll', '-1.570',  # 对应 RPY 中的第 1 个数
                '--frame-id', 'map',
                '--child-frame-id', 'camera_link'
            ],
            output='screen'
        ),

    ])