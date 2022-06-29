cmake_minimum_required(VERSION 3.0.2)
project(ekf_localization)

find_package(catkin REQUIRED COMPONENTS
  fpn_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES as_ekf
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ekf_node 
  src/ekf_node.cpp
  src/ekf_wrapper.cpp
  include/ekf/ekf.cpp
  include/ekf/model.cpp
)

add_dependencies(ekf_node fpn_msgs_generate_messages_cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES} )
