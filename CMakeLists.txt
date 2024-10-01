cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(admittance_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  manipulator_kdl
  filters
)
find_package(
  Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES admittance_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(admittance_controller src/admittance_controller.cpp)
target_link_libraries(admittance_controller ${catkin_LIBRARIES})
add_dependencies(admittance_controller ${catkin_EXPORTED_TARGETS})

add_library(admittance_control src/admittance_control.cpp)
target_link_libraries(admittance_control ${catkin_LIBRARIES} admittance_controller)
add_dependencies(admittance_control ${catkin_EXPORTED_TARGETS})

add_executable(admittance_controller_node src/admittance_control.cpp src/admittance_control_node.cpp)
target_link_libraries(admittance_controller_node ${catkin_LIBRARIES} admittance_control)
add_dependencies(admittance_controller_node ${catkin_EXPORTED_TARGETS})

# add_executable(admittance_controller_node src/admittance_control.cpp src/admittance_control_node.cpp)
# add_executable(admittance_controller_node src/admittance_control_node.cpp)
# target_link_libraries(admittance_controller_node ${catkin_LIBRARIES} admittance_controller rc_filter)
# add_dependencies(admittance_controller_node ${catkin_EXPORTED_TARGETS} admittance_controller)