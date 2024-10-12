# Admittance_controller

[[Ars Control Lab page]](https://www.arscontrol.unimore.it/)
[[Andrea Pupa]](https://www.arscontrol.unimore.it/andrea-pupa/)

This library provides the interface between the robot model and a custom admittance controller implementation. 

## Getting started

### Installation

Prerequisites: UBUNTU 20.04 - ROS NOETICS. It's also adviced to create a ssh key to your GitHub account. 

Firstly, setup your personal catkin workspace. You can manually create it from scratch as follows:

    mkdir -p your_catkin_ws/src
    cd catkin_ws
    catkin build

Secondly, setup the following dependencies (it's suggested to follow the installation instructions of their repository to avoid any issue):

    cd src/
    git@github.com:ARSControl/manipulator_kdl.git    
    git@github.com:orocos/orocos_kinematics_dynamics.git

Extract the folder "orocos_kdl" from "orocos_kinematics_dynamics" and delete the remaining folders and files on this repo. Then you can clone this repo:

    git clone git@github.com:ARSControl/admittance_controller.git

Compile the repo:

    cd your_catkin_ws
    catkin build

Alternatively, you can compile using the command

    catkin_make (or catkin_make_isolated)

## How to use

TODO

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
