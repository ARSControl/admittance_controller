cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(admittance_controller)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(filters_c REQUIRED)  # local package in workspace

# Include directories
include_directories(
  include
)

# Library target
add_library(admittance_controller SHARED
  src/admittance_controller.cpp
)

# --------- ADMITTANCE CONTROLLER ---------
ament_target_dependencies(admittance_controller
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  Eigen3
  filters_c
)

# --------- ADMITTANCE CONTROL NODE ---------
add_executable(admittance_control_node
  src/admittance_control.cpp
  src/admittance_control_node.cpp
)

ament_target_dependencies(admittance_control_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  Eigen3
  filters_c
)

target_link_libraries(admittance_control_node admittance_controller)

# Install targets
install(TARGETS
  admittance_controller
  admittance_control_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------------- WRENCH PLOTTER --------------
add_executable(wrench_plot_publisher src/wrench_plot_publisher.cpp)
ament_target_dependencies(wrench_plot_publisher rclcpp std_msgs geometry_msgs)
install(TARGETS
  wrench_plot_publisher
  DESTINATION lib/${PROJECT_NAME})


# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install additional files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  Eigen3
  filters_c
)
ament_export_include_directories(include)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()