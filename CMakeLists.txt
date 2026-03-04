cmake_minimum_required(VERSION 3.8)
project(admittance_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(filters_c REQUIRED)
find_package(manipulator_kdl REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${manipulator_kdl_INCLUDE_DIRS}
)

add_library(admittance_controller SHARED
  src/admittance_controller.cpp
)

target_link_libraries(admittance_controller PUBLIC
  rclcpp::rclcpp
  Eigen3::Eigen
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${filters_c_TARGETS}
  ${manipulator_kdl_TARGETS}
)

add_executable(admittance_control_node
  src/admittance_control.cpp
  src/admittance_control_node.cpp
)

target_link_libraries(admittance_control_node PRIVATE
  admittance_controller
)

install(TARGETS admittance_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS admittance_controller
  EXPORT export_admittance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_admittance_controller HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs geometry_msgs Eigen3 filters_c manipulator_kdl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()