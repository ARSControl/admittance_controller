/*
	MIT License

	Copyright (c) [2024] [Andrea Pupa] [Italo Almirante]

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

// IMPORT LIBRARIES
#include "admittance_control/admittance_control.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------
double AdmittanceControl::mean = 0.0;   // Mean static variable for the control time computation

// Constructor for the AdmittanceControl class
AdmittanceControl::AdmittanceControl()
{
    // Subscribe to joint states and force sensor topics
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AdmittanceControl::jointCallback, this);
    force_sub_ = nh_.subscribe("/ur_rtde/ft_sensor", 1, &AdmittanceControl::forceSensorCallback, this);

    // Advertise joint velocity command topic
    joint_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command", 1, true);

    // Advertise service to enable admittance control
    adm_service_ = nh_.advertiseService("/enable_admittance", &AdmittanceControl::enableAdmittance, this);

    // Create service client to zero the force-torque sensor
    ft_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_rtde/zeroFTSensor");

    // Initialize wrench to zero
    wrench_.setZero();

    // Define desired mass, damping, and stiffness matrices
    Eigen::MatrixXd Mdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Bdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Kdes = Eigen::MatrixXd::Zero(6, 6);
    std::string manipulator_name = "ur5e";
    int n_joints = 6;
    double ts = 0.002;
    std::vector<double> mass = {2.5, 2.5, 2.5, 0.05, 0.05, 0.05};
    std::vector<double> damping = {5.0, 5.0, 5.0, 0.3, 0.3, 0.3};

    // Initialize mass and damping matrices
    for (uint i = 0; i < 6; i++)
    {
        Mdes(i, i) = 5.0 * mass[i];
        Bdes(i, i) = 5.0 * damping[i];
    }

    // Create an instance of AdmittanceController
    adm_controller_ = new AdmittanceController(Mdes, Kdes, Bdes, manipulator_name, n_joints, ts);

    // Initialize wrench to zero
    wrench_.setZero();
}

// Callback function for joint states
void AdmittanceControl::jointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    // Update joint positions in the admittance controller
    adm_controller_->updateJoints(js->position);
}

// Callback function for force sensor data
void AdmittanceControl::forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w)
{
    // Update wrench values with force and torque data
    wrench_(0, 0) = w->force.x;
    wrench_(1, 0) = w->force.y;
    wrench_(2, 0) = w->force.z;
    wrench_(3, 0) = w->torque.x;
    wrench_(4, 0) = w->torque.y;
    wrench_(5, 0) = w->torque.z;
}

// Service callback to enable or disable admittance control
bool AdmittanceControl::enableAdmittance(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
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

// Main loop to compute and publish joint velocities
void AdmittanceControl::spinner()
{
    // Number of samples for mean computation
    unsigned long long int k = 0;           
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, shutdown_handler);

    // Set ROS rate
	ros::Rate r(500);

    // ROS loop
	while (ros::ok())
	{
        // Listen to callbacks
        ros::spinOnce();

        // Start loop time measurement
        ros::Time start = ros::Time::now();

        // Execute the main computations of the admittance control
        admittance_control_main();

        // Update mean computation
        // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());
        double sample_k = ros::Time::now().toSec()-start.toSec();
        k++;
        mean = 1/(static_cast<double>(k))*(sample_k+mean*static_cast<double>(k-1));

        // Wait for the next iteration
		r.sleep();
	}
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

// Main control loop function
void AdmittanceControl::admittance_control_main()
{
    // Compute the speed of the robot according to the given wrench
    auto dq = adm_controller_->computeSpeed(wrench_);
    // Convert the vel msg as ROS msg
    std_msgs::Float64MultiArray joint_vel;
    for (uint i = 0; i < dq.rows(); i++)    {joint_vel.data.push_back(dq(i, 0));}
    // Send the command to the robot
    joint_vel_pub_.publish(joint_vel);
}