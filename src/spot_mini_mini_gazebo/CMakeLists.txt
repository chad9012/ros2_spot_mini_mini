cmake_minimum_required(VERSION 3.8)
project(spot_mini_mini_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(spot_mini_mini_description REQUIRED)
find_package(spot_mini_mini_interfaces REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    config
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/hold_position.py
  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
