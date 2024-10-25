cmake_minimum_required(VERSION 3.8)
project(Sprint4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Include directories 
include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(circle_the_circle src/sprint.cpp)

# Link libraries
ament_target_dependencies(circle_the_circle rclcpp sensor_msgs geometry_msgs nav_msgs map_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs nav2_msgs rclcpp_action)

# Install executables
install(TARGETS
  circle_the_circle
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()


