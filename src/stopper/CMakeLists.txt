cmake_minimum_required(VERSION 3.5)
project(stopper)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(stopper_node src/stopper_node.cpp)
ament_target_dependencies(stopper_node rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

install(TARGETS
  stopper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
