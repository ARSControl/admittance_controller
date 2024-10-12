/************************************************************************************
 * MIT License
 * 
 * Copyright (c) 2024 [Andrea Pupa] [Italo Almirante]
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ************************************************************************************/

#include "admittance_control/admittance_control.h"

// --------------------- STATIC VARIABLES ---------------------
double AdmittanceControl::mean = 0.0;  // variable for the control time average computation

// --------------------- PUBLIC CONSTRUCTOR ---------------------
AdmittanceControl::AdmittanceControl(std::string node_name)
{
    // Update node params
    node_name_ = node_name;
    check_params();

    // Subscribe to topics
    joint_sub_ = nh_.subscribe("/joint_states",             1, &AdmittanceControl::jointCallback,       this);
    force_sub_ = nh_.subscribe(force_feed_topic_,           1, &AdmittanceControl::forceSensorCallback, this);
    xd_sub_    = nh_.subscribe(manipulator_name_+"/adm_xd", 1, &AdmittanceControl::admittanceXdCallback, this);

    // Load and apply parameters
    if (mode == "kdl")
    {
        vel_pub_ = nh.advertise<std_msgs::Float64MultiArray>(command_topic_, 1);
    }
    else
    {
        vel_pub_      = nh.advertise<geometry_msgs::Twist>(command_topic_, 1);
        tcp_pose_sub_ = nh_.subscribe(manipulator_name_+"/tcp_pose",1,&AdmittanceControl::tcpPoseCallback,this);
    }

    // Advertise service to enable admittance control
    adm_service_ = nh_.advertiseService(manipulator_name_+"/enable_admittance", &AdmittanceControl::enableAdmittance, this);

    // Create service client to zero the force-torque sensor
    ft_client_ = nh_.serviceClient<std_srvs::Trigger>(zero_ft_sensor_topic_);

    // Initialize wrench to zero
    wrench_.setZero();
}

// Node params update
void AdmittanceControl::check_param()
{
    // Init joints
    if (!nh.getParam(node_name_+"joint_names", joint_names_))
    {
        ROS_WARN("Joint names not set, using default names.");
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}; // Default UR5 joints
    }
    n_joints_ = joint_names_.size();

    // Init motion params
    std::vector<double> m_d, k_d, b_d;

    if (!nh.getParam(node_name_+"m_d", m_d))
    {
        ROS_WARN("Diagonal mass 'm_d' not set, using default value of 1.0.");
        m_d = Eigen::VectorXd::Constant(n_joints_, 1.0);
    }
    if (!nh.getParam(node_name_+"k_d", k_d))
    {
        ROS_WARN("Diagonal spring constant 'k_d' not set, using default value of 100.0.");
        k_d = Eigen::VectorXd::Constant(n_joints_, 100.0);
    }
    if (!nh.getParam(node_name_+"b_d", b_d))
    {
        ROS_WARN("Diagonal damping constant 'b_d' not set, using default value of 10.0.");
        b_d = Eigen::VectorXd::Constant(n_joints_, 10.0);
    }

    // Fill matrices values
    Eigen::MatrixXd<double,6,6> M_des = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd<double,6,6> K_des = Eigen::MatrixXd::Zero(6,6);
    Eigen::MatrixXd<double,6,6> B_des = Eigen::MatrixXd::Zero(6,6);

    for (uint i = 0; i < 6; i++)
    {
        M_des_(i, i) = m_d[i];
        K_des_(i, i) = k_d[i];
        B_des_(i, i) = b_d[i];
    }

    // Create an instance of AdmittanceController
    adm_controller_ = new AdmittanceController( Mdes, Kdes, Bdes, 1/ros_loop_);

    // Init interaction params
    if (!nh.getParam(node_name_+"/force_limit", force_limit_))
    {
        ROS_WARN("Force limit not set, using default values.");
        force_limit_ = Eigen::VectorXd::Constant(6, 10.0);
    }
    if (!nh.getParam(node_name_+"/mode", mode_))
    {
        ROS_WARN("Mode param not set, using default: kdl.");
        mode_ = "kdl";
    }
    if (mode_ == "kdl") {mode_bool_ = false; robot_kdl_ = new ManipulatorKDL(manipulator_);}
    else                {mode_bool_ =  true;}

    // Init control params
    if (!nh.getParam(node_name_+"/loop_rate", loop_rate_))
    {
        ROS_WARN("Loop rate not set, using default: 500 Hz.");
        loop_rate_ = 500.0; // Default 500 Hz
    }

    // Init model params
    if (!nh.getParam(node_name_+"/manipulator", manipulator_))
    {
        ROS_WARN("Manipulator name not set, using default: ur5.");
        manipulator_name_ = "ur5";
    }

    if (!nh.getParam(node_name_+"/manipulator_name", manipulator_name_))
    {
        ROS_WARN("Manipulator name not set, using default: manipulator.");
        manipulator_name_ = "manipulator";
    }

    // Init command topic
    if (!nh.getParam(node_name_+"/command_topic", command_topic_))
    {
        ROS_WARN("Command topic param not set, using default: /ur_rtde/controllers/joint_velocity_controller/command.");
        command_topic_ = "/ur_rtde/controllers/joint_velocity_controller/command";
    }
    // Init force feedback topic
    if (!nh.getParam(node_name_+"/force_feed_topic", force_feed_topic_))
    {
        ROS_WARN("Force feedback topic param not set, using default: /ur_rtde/ft_sensor.");
        force_feed_topic_ = "/ur_rtde/ft_sensor";
    }
    // Zero force feed server
    if (!nh.getParam(node_name_+"/zero_ft_sensor_topic", zero_ft_sensor_topic_))
    {
        ROS_WARN("Zero force feedback server name param not set, using default: /ur_rtde/zeroFTSensor");
        zero_ft_sensor_topic_ = "/ur_rtde/zeroFTSensor";
    }
}

// ---------------------------- UTILS ---------------------------------
// --------------------- QUATERNIONS HANDLER -------------------
// Conversion from degrees euler angles to quaternion
geometry_msgs::Quaternion ManipulatorMenu::quaternion_from_euler(double roll, double pitch, double yaw)
{
  // Declaration of empty quaternion
  geometry_msgs::Quaternion quaternion;

  // Conversion from euler rotation to pose quaternion
  tf2::Quaternion quat; quat.setRPY(roll*M_PI/180,pitch*M_PI/180,yaw*M_PI/180); quat.normalize();
  quaternion.x = quat.getX();
  quaternion.y = quat.getY();
  quaternion.z = quat.getZ();
  quaternion.w = quat.getW();

  return quaternion;
}
// Conversion from quaternion to degrees euler angles
std::vector<double> ManipulatorMenu::euler_from_quaternion(const geometry_msgs::Quaternion quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);

  // Get Euler angles
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  // Store the angles in a vector
  std::vector<double> euler_angles = {roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI};

  // Check if angles are in the interval (-180,180]
  for (unsigned int k = 0; k < 3; k++)
  {
    if      (euler_angles[k] < -179.9999999999) {euler_angles[k] += 360.;}
    else if (euler_angles[k] > +180.          ) {euler_angles[k] -= 360.;}
  }

  return euler_angles; 
}

// --------------------------- POSITION CALLBACKS -------------------------------

// Robot state callback
void AdmittanceControl::tcpPoseCallback(const geometry_msgs::Pose& msg)
{
    ee_pose_(0) = msg->pose.position.x;
    ee_pose_(1) = msg->pose.position.y;
    ee_pose_(2) = msg->pose.position.z;
    std::vector<double> ee_pose_rpy = euler_from_quaternion(msg->pose.orientation);
    ee_pose_(3) = ee_pose_rpy[0];
    ee_pose_(4) = ee_pose_rpy[1];
    ee_pose_(5) = ee_pose_rpy[2];

    // if (mode_bool_ == false) // If manipulator_kdl has been chosen as planning interface
    // {
    //     robot_kdl->fk(q, ee_pose_);
    // }
}

// Joint state callback
void AdmittanceControl::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // adm_controller_->updateJoints(msg->position);
    q_(0) = msg->position[0];
    q_(1) = msg->position[1];
    q_(2) = msg->position[2];
    q_(3) = msg->position[3];
    q_(4) = msg->position[4];
    q_(5) = msg->position[5];
}

// --------------------------- JACOBIAN COMPUTATIONS -----------------------------------
    if (mode_bool_ == false) // If manipulator_kdl has been chosen as planning interface
    {
        robot_kdl->jac(q, jacobian_);
        robot_kdl->fk(q, p_real_);
    }


// ----------------------------- SPEED COMPUTATIONS -----------------------------------


// SENSORS 

// Callback function for force sensor data
void AdmittanceControl::forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w)
{
    // Update wrench values with force and torque data
    wrench_(0) = w->force.x;
    wrench_(1) = w->force.y;
    wrench_(2) = w->force.z;
    wrench_(3) = w->torque.x;
    wrench_(4) = w->torque.y;
    wrench_(5) = w->torque.z;
}

//  SETPOINT UPDATE 
admittanceXdCallback

// ADMITTANCE ENABLING/DISABLING

// Service callback to enable or disable admittance control
bool AdmittanceControl::enableAdmittance(std_srvs::SetBool::Request  &req,
                                         std_srvs::SetBool::Response &res)
{
    if (req.data)
    {
        // Enable admittance control and zero the force-torque sensor
        adm_controller_->enableAdmittance();
        std_srvs::Trigger srv;
        ft_client_.call(srv);
        ros::spinOnce();
    }
    else
    {
        // Disable admittance control
        adm_controller_->disableAdmittance();
    }
    res.success = true;
    return true;
}

// Shutdown handler
void AdmittanceControl::shutdown_handler(int sig)
{
  // Show the result of the admittance control mean duration
  ROS_INFO("Mean duration of the admittance control computations: %f", mean);
  ros::Duration(1.0).sleep();

  // Shutdown ROS
  ros::shutdown();
}

// Main ROS spinner
void AdmittanceControl::spinner()
{
    // Number of samples for mean computation
    unsigned long long int k = 0;           
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, shutdown_handler);

    // Set the loop rate
    ros::Rate rate(loop_rate_);

    // ROS loop
	while (ros::ok())
	{
        // Start loop time measurement
        ros::Time start = ros::Time::now();

        // Listen to callbacks
        ros::spinOnce();

        // Execute the main computations of the admittance control
        admittance_control_main();

        // Update mean computation
        // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());
        double sample_k = ros::Time::now().toSec()-start.toSec(); k++;
        mean = 1/(static_cast<double>(k))*(sample_k+mean*static_cast<double>(k-1));

        // Wait for the next iteration
		r.sleep();
	}
}

// Main control loop function
void AdmittanceControl::admittance_control_main()
{
    if (mode_ == "kdl")
    {
        // Compute the speed of the robot according to the given wrench
        Eigen::VectorXd::Zero(n_joints_) dq = adm_controller_->computeSpeed(wrench_);

        // Convert the vel msg as ROS msg
        std_msgs::Float64MultiArray joint_vel;
        for (uint i = 0; i < n_joints_; i++)    {joint_vel.data.push_back(dq(i));}

        // Send the command to the robot
        vel_pub_.publish(joint_vel);
    }
    else
    {
        // Compute the speed of the robot according to the given wrench
        Eigen::VectorXd::Zero(n_joints_) dx = adm_controller_->computeEESpeed(wrench_);

        // Convert the vel msg as ROS msg
        geometry_msgs::Twist ee_vel;
        ee_vel.linear.x  = dx(0);
        ee_vel.linear.y  = dx(1);
        ee_vel.linear.z  = dx(2);
        ee_vel.angular.x = dx(3);
        ee_vel.angular.y = dx(4);
        ee_vel.angular.z = dx(5);

        // Send the command to the robot
        vel_pub_.publish(ee_vel);
    }
}