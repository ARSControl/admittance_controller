# Admittance_controller

This library provides the interface between the robot model and a custom admittance controller implementation. 

## Getting started

### Prerequisites - UBUNTU 20.04 - ROS NOETICS

Firstly, setup your personal catkin workspace. You can manually create it from scratch as follows:

    mkdir -p catkin_ws/src
    cd catkin_ws
    catkin build

Secondly, setup the following dependencies (it's suggested to follow the installation instructions of their repository to avoid any issue):

    git@github.com:ARSControl/manipulator_kdl.git    
    git@github.com:orocos/orocos_kinematics_dynamics.git

Extract the folder "orocos_kdl" from "orocos_kinematics_dynamics" and delete the remaining folders and files on this repo.

## How to use

## Authors

   Andrea Pupa

## Issues

Please if you have any issue in compiling the nodes or using it, you can open an issue on git or send me an email at my email address: andrea.pupa@unimore.it.
