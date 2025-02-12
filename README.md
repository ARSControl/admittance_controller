# Admittance_controller

[[Ars Control Lab page]](https://www.arscontrol.unimore.it/)
[[Andrea Pupa]](https://www.arscontrol.unimore.it/andrea-pupa/)
[[Italo Almirante]](https://www.arscontrol.unimore.it/italo-almirante/)

This library provides the interface between the robot model and a custom admittance controller implementation. 

## Getting started

### Installation

Prerequisites: UBUNTU 22.04 - ROS HUMBLE. It's also adviced to create a ssh key to your GitHub account. 

Firstly, setup your personal colcon workspace. You can manually create it from scratch as follows:

    mkdir -p your_colcon_ws/src
    cd colcon_ws
    colcon build

Secondly, setup the following dependencies (it's suggested to follow the installation instructions of their repository to avoid any issue):

    cd src/
    git@github.com:ARSControl/manipulator_kdl.git    
    git@github.com:orocos/orocos_kinematics_dynamics.git

Extract the folder "orocos_kdl" from "orocos_kinematics_dynamics" and delete the remaining folders and files on this repo. Then you can clone this repo:

    git clone git@github.com:ARSControl/admittance_controller.git

To interface the repo with the manipulator planner, copy the following command to the terminal and follow its installation instructions:

    git clone git@github.com:Italo-99/manipulators.git

Compile the repo:

    cd your_colcon_ws
    colcon build

## How to use

Use the file "admittance_control_params.yaml" to setup the configuration of your robot. The meaning of each param is accurately described in this config file.

The main executable node is "admittance_control_node". To automatically launch it, with your custom configuration, run the following command on the terminal:

    ros2 launch admittance_controller admittance_control.launch

If you want to interface the robot with the library "manipulator_kdl", launch the command (as ur5e example):

    ros2 launch manipulator_kdl manipulator_kdl.launch robot_name:=ur5

If you want to interface the robot with MoveIt!, launch the command (as ur5e example):

    ros2 launch manipulators ur5e_planner.launch

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