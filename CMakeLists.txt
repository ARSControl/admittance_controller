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
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

############################################################
################## SET INCLUDE PATH ########################
############################################################
# Include directories
include_directories($(CCD_INCLUDE_DIRS))
include_directories($(CMAKE_CURRENT_SOURCE_DIR)/include)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")
include_directories(/include)

############################################################
################## SET LIBRARIES PATH ######################
############################################################
# link directory e.g. link_directories( usr/lib/x86_64-linux-gnu )

############################################################
################## LINK LIBRARIES ##########################
############################################################
# link libraries e.g. link_libraries(${ExtPkg_LIBS})

############################################################
################## EXECUTABLES #############################
############################################################

### admittance_controller library 
add_executable(
  admittance_controller src/admittance_controller.cpp
  include/admittance_controller/admittance_controller.hpp
)
ament_target_dependencies( admittance_controller rclcpp filters std_msgs geometry_msgs )
target_include_directories( admittance_controller PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)
target_link_libraries( admittance_controller fcl ${CDD_LIBRARIES} )
target_compile_features( admittance_controller PUBLIC c_std_99 cxx_std_17 )

### admittance_control library 
add_executable(
  admittance_control src/admittance_control.cpp
  include/admittance_control/admittance_control.hpp
)
ament_target_dependencies( admittance_control rclcpp std_msgs geometry_msgs sensor_msgs )
target_include_directories( admittance_control PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)
target_link_libraries( admittance_control fcl ${CDD_LIBRARIES} )
target_compile_features( admittance_control PUBLIC c_std_99 cxx_std_17 )

### admittance_control_node library 
add_executable(
  admittance_control_node src/admittance_control_node.cpp
  include/admittance_control/admittance_control.hpp
)
ament_target_dependencies( admittance_control_node rclcpp geometry_msgs )
target_include_directories( admittance_control_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)
target_link_libraries( admittance_control_node fcl ${CDD_LIBRARIES} )
target_compile_features( admittance_control_node PUBLIC c_std_99 cxx_std_17 )


############################################################
################## INSTALLING ##############################
############################################################
# install nodes
install(
  TARGETS
    admittance_controller
    admittance_control
    admittance_control_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# install launch and/or config files in share path of package
install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)


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

target_link_libraries( admittance_controller "${cpp_typesupport_target}" )
target_link_libraries( admittance_control "${cpp_typesupport_target}" )
target_link_libraries( admittance_control_node "${cpp_typesupport_target}" )


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