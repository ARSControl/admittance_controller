# Admittance_controller

[[Ars Control Lab page]](https://www.arscontrol.unimore.it/)
[[Andrea Pupa]](https://www.arscontrol.unimore.it/andrea-pupa/)
[[Italo Almirante]](https://www.arscontrol.unimore.it/italo-almirante/)

This library provides the interface between the robot model and a custom admittance controller implementation. 

## Getting started

### Installation

Prerequisites: UBUNTU 22.04 - ROS HUMBLE. It's also adviced to create a ssh key to your GitHub account. 

Firstly, clone this repo with the following command:

    cd your_ws/src/
    git clone git@github.com:ARSControl/admittance_controller.git -b ros2humble_moveit

To interface the repo with the manipulator planner, copy the following command to the terminal and follow its installation instructions:

    git clone git@github.com:Italo-99/manipulators.git

Compile the repo:

    cd your_catkin_ws
    colcon build --packages-select admittance_controller --symlink-install 

## How to use

Launch the planner and the robot model using:

    ros2 launch manipulators planner.launch.py ur_type:=ur5e
    ros2 launch manipulators planner.launch.py ur_type:=ur10e

You can use the manipulator menu to easily handle the robot:

    ros2 run manipulators manipulator_menu_user

Use the file "admittance_control_params.yaml" to setup the configuration of your robot. The meaning of each param is accurately described in this config file.

The main executable node is "admittance_control_node". To automatically launch it, with your custom configuration, run one of the following command on the terminal:

    ros2 launch admittance_controller ur5e.launch.py
    ros2 launch admittance_controller ur10e.launch.py

To run the plotter of the manipulator forces:

    ros2 run admittance_controller wrench_plot_publisher

To run the admittance menu:

    ros2 run admittance_controller admittance_menu_node

### Optimized compiler

If you want to speed your code running performances, you can add the following lines to the "CMakeLists".txt of this package:

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

Or alternatively:

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

Your code should run faster, despite the longer compiler time needed. By default, O3 is set. You can maually comment it, but it's strongly suggested to use it.

## Issues

Please if you have any issue in compiling the nodes or using them, you can open an issue on git or send me an email at my email address: andrea.pupa@unimore.it.