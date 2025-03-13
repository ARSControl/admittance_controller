cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(admittance_controller)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

############################################################
################## SET CMAKE OPTIONS #######################
############################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


############################################################
################## INCLUDE CMAKE FILES #####################
############################################################


############################################################
################## FIND PACKAGES ###########################
############################################################
find_package(ament_cmake REQUIRED)
find_package(filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manipulator_kdl REQUIRED)
find_package(energy_tank REQUIRED)

############################################################
################## SET INCLUDE PATH ########################
############################################################
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${manipulator_kdl_INCLUDE_DIRS}
)


############################################################
################## LIBRARIES ##########################
############################################################
add_library(admittance_controller SHARED src/admittance_controller.cpp)
ament_target_dependencies(admittance_controller rclcpp std_msgs geometry_msgs filters Eigen3 std_srvs manipulator_kdl)

# add_library(admittance_control SHARED src/admittance_control.cpp)
# ament_target_dependencies(admittance_control rclcpp std_msgs manipulator_kdl filters Eigen3 std_srvs)
# target_link_libraries(admittance_control admittance_controller)


############################################################
################## EXECUTABLES #############################
############################################################
add_executable(admittance_control_node src/admittance_control.cpp src/admittance_control_node.cpp)
ament_target_dependencies(admittance_control_node rclcpp std_msgs manipulator_kdl filters Eigen3 energy_tank)
target_link_libraries(admittance_control_node admittance_controller)


############################################################
################## INSTALLING ##############################
############################################################
# Install targets
install(
  TARGETS 
    admittance_control_node
  DESTINATION 
    lib/${PROJECT_NAME}
)

install(
  TARGETS 
    admittance_controller
  EXPORT export_admittance_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(
#   TARGETS 
#     admittance_control 
#   EXPORT export_admittance_control
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(
  DIRECTORY
    launch
    config
  DESTINATION 
    share/${PROJECT_NAME}
)

install(
  DIRECTORY 
    include/
  DESTINATION
    include
)

# Export package dependencies
ament_export_libraries(admittance_controller) # admittance_control)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs manipulator_kdl filters Eigen3)

############################################################
################## INTERFACES ##############################
############################################################
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/example.srv"
#   "msg/message.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )
# ament_export_dependencies(rosidl_default_runtime)

# rosidl_get_typesupport_target( cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp
# )

target_link_libraries(admittance_controller "${cpp_typesupport_target}")
# target_link_libraries(admittance_control "${cpp_typesupport_target}")

############################################################
################## BUILD TESTING ###########################
############################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()