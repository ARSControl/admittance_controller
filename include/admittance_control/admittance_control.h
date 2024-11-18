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

#ifndef ADMITTANCE_CONTROL_H
#define ADMITTANCE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <Eigen/Dense>
#include <signal.h>
#include "admittance_controller/admittance_controller.h"
#include <manipulator_kdl/manipulator_kdl.h>

class AdmittanceControl
{
public:
    AdmittanceControl(const std::string& node_name);
    
    void spinner();
      
private:
    
	// ------------------------------ FUNCTIONS ----------------------------
	
	static void shutdown_handler(int sig);           // ROS shutdown handler
    
    // Callback functions
    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void tcpPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void tcpTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w);
    void admittanceXdCallback(const geometry_msgs::Pose::ConstPtr &p);

    Eigen::VectorXd wrenchFilter(const Eigen::VectorXd& wrench);
    
    // Main control function
    void admittance_control_main();
    
    // Service callback to enable or disable admittance control
    bool enableAdmittance(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool enablePushRegulation(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  
	// Params handler
	void check_params();                              // Node params update

	// Jacobian hanlders
    Eigen::MatrixXd getJacobian();                   // Get Jacobian
    Eigen::MatrixXd getInvJacobian();                // Get Inverse Jacobian

	// Utils
	Eigen::Quaterniond quaternion_from_euler(const double& roll, const double& pitch, const double& yaw);
	Eigen::Vector3d euler_from_quaternion(const Eigen::Quaterniond& quaternion);

    // Variable admittance
    void changeMassAdmittanceCallback(      const geometry_msgs::Vector3::ConstPtr& new_params);
    void changeDampAdmittanceCallback(      const geometry_msgs::Vector3::ConstPtr& new_params);
    void changeStiffAdmittanceCallback(     const geometry_msgs::Vector3::ConstPtr& new_params);
    void changeMassRotAdmittanceCallback(   const geometry_msgs::Vector3::ConstPtr& new_params);
    void changeDampRotAdmittanceCallback(   const geometry_msgs::Vector3::ConstPtr& new_params);
    void changeStiffRotAdmittanceCallback(  const geometry_msgs::Vector3::ConstPtr& new_params);

	// ------------------------------ VARIABLES ----------------------------

	// Static variables
    static double mean;                              // Variable for the control time average computation

    // Node parameters
    std::string node_name_;
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_, force_sub_, xd_sub_, tcp_pose_sub_, tcp_twist_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher wf_pub_;
    ros::ServiceServer adm_service_;
    ros::ServiceServer push_service_;
    ros::ServiceClient ft_client_;

    // Topics and other parameters
    std::string command_topic_, force_feed_topic_, zero_ft_sensor_topic_, ee_pose_topic_, ee_vel_topic_;
	std::string manipulator_name_, manipulator_;
    double loop_rate_;
    int    n_joints_;
    std::vector<std::string> joint_names_;
    
    // Controller instance
    AdmittanceController* adm_controller_;
    ros::Subscriber m_adm_pos_sub_;
    ros::Subscriber b_adm_pos_sub_;
    ros::Subscriber k_adm_pos_sub_;
    ros::Subscriber m_adm_rot_sub_;
    ros::Subscriber b_adm_rot_sub_;
    ros::Subscriber k_adm_rot_sub_;
    
    // KDL instance (for mode "kdl")
    ManipulatorKDL* robot_kdl_;

    // Wrench filter
    filters::RCFilter* force_filter_;
    
    // Matrices and vectors for control
    Eigen::VectorXd q_, xd_, dx_, dx_des_, ddx_des_, wrench_, ee_pose_;
    Eigen::VectorXd force_limit_;
	double kp_pos_, kp_rot_, dz_force_, dz_torque_;
    
	// Mode selection flag
    bool mode_bool_;
    std::string mode_;
};

#endif // ADMITTANCE_CONTROL_H
