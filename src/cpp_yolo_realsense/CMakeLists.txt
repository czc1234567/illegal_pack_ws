cmake_minimum_required(VERSION 3.8)
project(cpp_yolo_realsense)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找 ROS 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. 查找外部依赖 (OpenCV, CUDA, TensorRT)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# 设置 Jetson 上的库路径
include_directories(
  include
  /usr/include/aarch64-linux-gnu 
  /usr/local/cuda/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/aarch64-linux-gnu
  /usr/local/cuda/lib64
)

# 3. 编译节点
add_executable(rs_yolo_node src/rs_yolo_node.cpp)

# 4. 链接库
ament_target_dependencies(rs_yolo_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
  visualization_msgs
)

target_link_libraries(rs_yolo_node
  ${OpenCV_LIBS}
  nvinfer
  cudart
)

# 5. 安装
install(TARGETS rs_yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

# 拷贝 launch 或 params (如果有)
install(DIRECTORY weights
  DESTINATION share/${PROJECT_NAME}
)

ament_package()