import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'realsense3d_detection'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 设置 RViz 配置文件路径
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'default.rviz')

    return LaunchDescription([
        # --- 节点 1: 地图服务器 ---
        Node(
            package=pkg_name,
            executable='simple_map_node',
            name='map_server',
            output='screen'
        ),

        # --- 节点 2: 静态坐标变换 (地图 -> 机器人底盘) ---
        # 这里的数值来自 tf2_echo map base_link 的结果
        # 含义：定义机器人底盘(base_link) 在地图(map) 中的位置
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base',
            arguments=[
                '--x', '-8.167',     # 刚才获取的 x
                '--y', '5.716',      # 刚才获取的 y
                '--z', '0.000',      # 地面高度通常为 0
                '--yaw', '2.988',    # 刚才获取的 yaw (朝向)
                '--pitch', '0.000',  # 地面通常是平的
                '--roll', '0.000',   # 地面通常是平的
                '--frame-id', 'map',          # 父坐标系 (世界)
                '--child-frame-id', 'base_link' # 子坐标系 (机器人本体)
            ],
            output='screen'
        ),


        # --- 节点 3: RViz2 (可选，如果不需要自动启动可注释) ---
        # Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_path],
        #    output='screen'
        # ),
    ])