cmake_minimum_required(VERSION 3.8)
project(Lab4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executables
add_executable(dead_reckoning src/lab.cpp)

# Link libraries
ament_target_dependencies(dead_reckoning rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# Install executables
install(TARGETS
  dead_reckoning
  DESTINATION lib/${PROJECT_NAME})

ament_package()
