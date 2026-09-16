cmake_minimum_required(VERSION 3.8)
project(perception_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dth_messages REQUIRED)

# 本包自身源码也要能 #include <perception_common/...>（对外由 ament_export_include_directories 提供）
include_directories(include)

# 共用头（仅头文件）：运动方向定向检测门控
#   #include <perception_common/motion_state.hpp>
# 由 pothole_detection（边坡）与 cluster（障碍物）两个检测包共用，见 install(DIRECTORY include/)
# 与 ament_export_include_directories()。

# 共用融合节点：/perception/edge_warning + /perception/obstacle_warning + /perception/stop_command
# 三路契约话题的唯一发布者——两个检测包的「单雷达原始告警」在此聚合
add_executable(warning_fusion_node
  src/warning_fusion_node.cpp
)

ament_target_dependencies(warning_fusion_node
  rclcpp
  geometry_msgs
  dth_messages
)

# Install
install(TARGETS
  warning_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 供检测包 ament_target_dependencies(perception_common) 使用
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs)

ament_package()
