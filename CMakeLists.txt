cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(admittance_controller)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- SET PKG DEPENDENCIES ----------------

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ur_rtde_controller REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(filters_c REQUIRED)  # local package in workspace
pkg_check_modules(SDL2 REQUIRED sdl2)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetFloat64.srv"
)

# Include directories
include_directories(
  include
)

# Library target
add_library(admittance_controller_lib SHARED
  src/admittance_controller.cpp
)

# --------- ADMITTANCE CONTROLLER ---------
ament_target_dependencies(admittance_controller_lib
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
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(admittance_control_node "${cpp_typesupport_target}")

target_link_libraries(admittance_control_node admittance_controller_lib)

# Install targets
install(TARGETS
  admittance_controller_lib
  admittance_control_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# -------------- WRENCH PLOTTER --------------
add_executable(wrench_plot_publisher src/wrench_plot_publisher.cpp)
ament_target_dependencies(wrench_plot_publisher rclcpp std_msgs geometry_msgs)
install(TARGETS
  wrench_plot_publisher
  DESTINATION lib/${PROJECT_NAME})

# ------------ ADMITTANCE MENU ------------
add_executable(admittance_menu_node src/admittance_menu_node.cpp)
ament_target_dependencies(admittance_menu_node rclcpp std_srvs std_msgs geometry_msgs)
install(TARGETS admittance_menu_node DESTINATION lib/${PROJECT_NAME})

# ------------ JOYSTICK MODE COMMANDER ------------
add_executable(joy_mode_command_node src/joy_mode_command_node.cpp)
ament_target_dependencies(joy_mode_command_node rclcpp sensor_msgs geometry_msgs std_srvs std_msgs)
install(TARGETS joy_mode_command_node DESTINATION lib/${PROJECT_NAME})

# ------------ ADMITTANCE GUI ------------
add_executable(admittance_gui_node src/admittance_gui_node.cpp)
ament_target_dependencies(admittance_gui_node rclcpp sensor_msgs geometry_msgs std_srvs ur_rtde_controller std_msgs)
ament_target_dependencies(admittance_gui_node neo_msgs2)
target_include_directories(admittance_gui_node PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(admittance_gui_node ${SDL2_LIBRARIES} "${cpp_typesupport_target}")
install(TARGETS admittance_gui_node DESTINATION lib/${PROJECT_NAME})

# -------------- INSTALL DIRECTORIES ------------
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

# ---------------- EXPORT DEPENDENCIES --------------

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
  rosidl_default_runtime
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
