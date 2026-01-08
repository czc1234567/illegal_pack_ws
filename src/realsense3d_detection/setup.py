from setuptools import find_packages, setup
import os
from glob import glob  # ✅ 核心修复1：新增glob导入（必须，否则glob函数报错）

# 你的包名（全小写，符合ROS2规范，无需修改）
package_name = 'realsense3d_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ✅ 核心修复2：统一用小写package_name（你定义的是小写，之前写大写PACKAGE_NAME会报错）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ✅ 新的写法 (拷贝 urdf 文件夹下的 所有文件，包括 yaml 和 pgm):
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RealSense 3D detection node for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_node = realsense3d_detection.detection_node:main',
            # ✅ 核心修复3：删除.py后缀（ROS2入口函数不能带文件后缀，致命语法错误）
            'CoordTransformer_node = realsense3d_detection.coordtansformer_node:main',
            'try2_node = realsense3d_detection.try2_node:main',
            'simple_map_node = realsense3d_detection.simple_map_node:main',
        ],
    },
)
