cmake_minimum_required(VERSION 3.5)
project(pothole_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(dth_messages REQUIRED)
# 共用件：运动方向门控头文件（<perception_common/motion_state.hpp>）
find_package(perception_common REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# 沟壑/悬崖检测节点（前/后雷达各启动一个实例，含运动方向定向检测门控）
# 只发布「单雷达原始告警」~/edge_warning_raw；契约话题 /perception/edge_warning 与
# /perception/stop_command 由 perception_common 包的 warning_fusion 节点统一发布
add_executable(pothole_detection_node
  src/pothole_detection_node.cpp
)

ament_target_dependencies(pothole_detection_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  dth_messages
  perception_common
)

target_link_libraries(pothole_detection_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS
  pothole_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
