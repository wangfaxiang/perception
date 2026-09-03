cmake_minimum_required(VERSION 3.5)
project(lidar_leveling)

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
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Build the node
add_executable(lidar_leveling_node
  src/lidar_leveling_node.cpp
)

ament_target_dependencies(lidar_leveling_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(lidar_leveling_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS lidar_leveling_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
