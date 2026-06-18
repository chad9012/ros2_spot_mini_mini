cmake_minimum_required(VERSION 3.8)
project(spot_mini_mini_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentData.msg"
  "msg/ContactData.msg"
  "msg/IMUdata.msg"
  "msg/JointAngles.msg"
  "msg/JointPulse.msg"
  "msg/JoyButtons.msg"
  "msg/MiniCmd.msg"
  "srv/CalibServo.srv"
)


ament_package()
