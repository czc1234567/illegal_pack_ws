from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动你的地图发布节点
        Node(
            package='realsense3d_detection', # 请确保包名正确
            executable='simple_map_node',    # 你 setup.py 里设置的入口点名称
            name='map_server'
        ),

        # 2. 启动静态坐标变换 (替代你刚才手敲的命令)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments = [
                '--x', '0', '--y', '0', '--z', '1.0',
                '--yaw', '-1.57', '--pitch', '0', '--roll', '-1.57',
                '--frame-id', 'map',
                '--child-frame-id', 'camera_link'
            ]
        ),
        
        # 3. (可选) 甚至可以直接在这里启动 RViz2
        # Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', '/path/to/your/config.rviz']
        # )
    ])