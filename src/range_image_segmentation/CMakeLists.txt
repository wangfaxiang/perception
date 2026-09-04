cmake_minimum_required(VERSION 3.5)
project(range_image_segmentation)

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
find_package(box_msg REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(${PCL_INCLUDE_DIRS})

# Build the node
add_executable(range_image_segmentation_node
  src/range_image_segmentation.cpp
)

ament_target_dependencies(range_image_segmentation_node
  rclcpp
  sensor_msgs
  visualization_msgs
  box_msg
  pcl_conversions
)

target_link_libraries(range_image_segmentation_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS range_image_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
