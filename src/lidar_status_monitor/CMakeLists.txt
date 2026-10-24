cmake_minimum_required(VERSION 3.8)
project(lidar_status_monitor)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息：雷达数据流状态
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarStatus.msg"
  DEPENDENCIES std_msgs
)

# 监测节点：订阅雷达点云，周期发布 LidarStatus
add_executable(lidar_status_monitor_node
  src/lidar_status_monitor_node.cpp
)

ament_target_dependencies(lidar_status_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
)

# 链接本包生成的消息库（供 include <lidar_status_monitor/msg/lidar_status.hpp>）
rosidl_target_interfaces(lidar_status_monitor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install
install(TARGETS
  lidar_status_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
