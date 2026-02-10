# Start communication with neobotics mobile robot (on-board)

Look at Neobotics_HWDoc.odt

    ssh neobotix@192.168.2.50
    (mpo-500, password: neobotix)
    ROS_AUTOSTART.sh

# Launch joystick device (on-device)
    
    ros2 run joy joy_node

## Launch joystick app (A-arm only, X-neobotics only, y-all, b-break)

    ros2 launch joy_mode_command.launch.py 

# Start neobotics remote controller (on-device test)

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Start communication with ur5e robot

    ros2 launch ur_rtde_controller rtde_controller.launch.py ROBOT_IP:=192.168.2.10 enable_gripper:=true

# Start ur5e planner and controller

    ros2 launch manipulators planner.launch.py ur_type:=ur5e
    ros2 run manipulators manipulator_menu_user

# Launch whole body controller

# Launch admittance controller

# Launch gripper controller (?)

# Launch app

## Launch admittance controller app

## Launch whole body controller app


## Launch joystick arm

## Launch arm app

## Launch neobotics mobile robot app