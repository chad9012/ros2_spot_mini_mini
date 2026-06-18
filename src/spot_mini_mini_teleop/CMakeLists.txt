cmake_minimum_required(VERSION 3.8)
project(spot_mini_mini_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(spot_mini_mini_interfaces REQUIRED)

include_directories(include)

# ── teleop_node ───────────────────────────────────────────────────────
# Reads Xbox controller and publishes Twist + JoyButtons + estop
add_executable(teleop_node
  src/teleop_node.cpp
  src/teleop.cpp
)
ament_target_dependencies(teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  spot_mini_mini_interfaces
)

# ── spot_sm_node ──────────────────────────────────────────────────────
# State machine: receives Twist, outputs MiniCmd to policy runner
# spot.cpp is pure logic — no ROS dependency, compiled into this node
add_executable(spot_sm_node
  src/spot_sm.cpp
  src/spot.cpp
)
ament_target_dependencies(spot_sm_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  spot_mini_mini_interfaces
)

install(TARGETS
  teleop_node
  spot_sm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY policies
  DESTINATION share/${PROJECT_NAME}
)

ament_package()