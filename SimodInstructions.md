# Start communication with neobotics mobile robot (on-board)

Look at Neobotics_HWDoc.odt. Run the following commands if auto start doesn't start alone:

    ssh neobotix@192.168.2.50
    (mpo-500, password: neobotix)
    source /opt/ros/humble/setup.bash
    pkill -f ros2
    ./ROS_AUTOSTART.sh

Otherwise, just check topics (at least /cmd_vel) and ping through:

    sshneo

If something doesn't work, try to launch directly the main neobotics code with:

    ros2 launch neo_mpo_500-2 bringup.launch.py

# Launch joystick device (on-device)
    
    ros2 run joy game_controller_node --ros-args -r /joy:=/mobile_platform/joy

## Launch joystick app (A-arm only, X-neobotics only, y-all, b-break)

    ros2 launch admittance_controller joy_mode_command.launch.py

# Start communication with ur5e robot

    ros2 launch ur_rtde_controller rtde_controller.launch.py ROBOT_IP:=192.168.2.30 enable_gripper:=true --ros-args -r /joint_state:=/fake/joint_state

# Start ur5e planner and controller

Launch the following cmd to enable the planner:

    ros2 launch manipulators planner.launch.py ur_type:=ur5e publish_joint_states:=False

Or you can directly use the following if you want to include gripper collision box and mobile robot platform collision boxes:

    ros2 launch manipulators custom_robot.launch.py ur_type:=ur5e robot:=ur5e_mobile publish_joint_states:=False gripper:=no_gripper parent_link:=world_ur
    xacro_args:='camera:=false gripper:=false gfloor:=true gripper_collision_box:=true tcp_offset:=0.148'

Launch the interaction menu (if you need further planning tools):

    ros2 run manipulators manipulator_menu_user

Launch the driver controller to connect to the robot:

    ros2 launch manipulators real_control_driver.launch.py ur_type:=ur5e

# Launch TCP-world pose publisher

This is the EE pose referred to the world frame, considered the integrated speed motion of the mobile base:

    ros2 run wbqp_controller tcp_pose_converter

# Launch whole body controller

    ros2 launch wbqp_controller wbqp_controller.launch.py

# Launch admittance controller

    ros2 launch admittance_controller mobile_ur5e.launch.py

# Display the interaction forces

    ros2 run admittance_controller wrench_plot_publisher --ros-args -p filtered_wrench_topic:=/mobile_manipulator/filtered_wrench

# Launch app

    ros2 run admittance_controller admittance_gui_node

# Old code prototype

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/mobile_manipulator/cmd_vel
    ros2 run sirio_utilities sirio_arm_gui --ros-args -r /manipulator/tcp_force:=/mobile_manipulator/filtered_wrench
    ros2 run admittance_controller admittance_menu_node --ros-args -p manipulator_name:=mobile_manipulator