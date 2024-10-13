cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(admittance_controller)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  manipulator_kdl
  filters
)
find_package(Eigen3 REQUIRED)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES admittance_controller
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build the admittance_controller library
add_library(admittance_controller src/admittance_controller.cpp)
target_link_libraries(admittance_controller ${catkin_LIBRARIES})
add_dependencies(admittance_controller ${catkin_EXPORTED_TARGETS})

# Build the admittance_control library
add_library(admittance_control src/admittance_control.cpp)
target_link_libraries(admittance_control ${catkin_LIBRARIES} admittance_controller)
add_dependencies(admittance_control ${catkin_EXPORTED_TARGETS})

# Build the node
add_executable(admittance_control_node src/admittance_control_node.cpp)
target_link_libraries(admittance_control_node ${catkin_LIBRARIES} admittance_control)
add_dependencies(admittance_control_node ${catkin_EXPORTED_TARGETS})
