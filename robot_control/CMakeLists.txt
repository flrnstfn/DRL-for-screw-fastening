cmake_minimum_required(VERSION 3.0.2)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  hardware_interface
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cartesian_impedance_controller
  message_runtime
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_control
  CATKIN_DEPENDS controller_manager effort_controllers hardware_interface roscpp rospy sensor_msgs std_msgs cartesian_impedance_controller message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../robot_control/include
)

#add_executable(doosan_hwi_node src/DoosanHWI.cpp)

# target_link_libraries(doosan_hwi_node
#   ${catkin_LIBRARIES}
# )

# install(TARGETS doosan_hwi_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

