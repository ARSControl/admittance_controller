/************************************************************************************
 * MIT License
 * 
 * Copyright (c) 2024 [Andrea Pupa] [Italo Almirante] [Matteo Nini]
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <eigen3/Eigen/Eigen>
#include <signal.h>
#include "admittance_controller/admittance_controller.hpp"
#include "manipulator_kdl/manipulator_kdl.hpp"

class AdmittanceControl : public rclcpp::Node
{
public:
    AdmittanceControl(const std::string& node_name);
    
    void spinner();
      
private:
    
	// ------------------------------ FUNCTIONS ----------------------------
	
	static void shutdown_handler(int sig);           // ROS shutdown handler
    
    // Callback functions
    void jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void tcpPoseCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);
    void tcpTwistCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
    void forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w);
    void admittanceXdCallback(const std::shared_ptr<geometry_msgs::msg::Pose> p);

    Eigen::VectorXd wrenchFilter(const Eigen::VectorXd& wrench);
    
    // Main control function
    void admittance_control_main();
    
    // Service callback to enable or disable admittance control
    bool enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> res);
    bool enablePushRegulation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  
	// Params handler
	void check_params();                              // Node params update

	// Jacobian hanlders
    Eigen::MatrixXd getJacobian();                   // Get Jacobian
    Eigen::MatrixXd getInvJacobian();                // Get Inverse Jacobian

	// Utils
	Eigen::Quaterniond quaternion_from_euler(const double& roll, const double& pitch, const double& yaw);
	Eigen::Vector3d euler_from_quaternion(const Eigen::Quaterniond& quaternion);

    // Variable admittance
    void changeMassAdmittanceCallback(      const geometry_msgs::msg::Vector3::SharedPtr new_params);
    void changeDampAdmittanceCallback(      const geometry_msgs::msg::Vector3::SharedPtr new_params);
    void changeStiffAdmittanceCallback(     const geometry_msgs::msg::Vector3::SharedPtr new_params);
    void changeMassRotAdmittanceCallback(   const geometry_msgs::msg::Vector3::SharedPtr new_params);
    void changeDampRotAdmittanceCallback(   const geometry_msgs::msg::Vector3::SharedPtr new_params);
    void changeStiffRotAdmittanceCallback(  const geometry_msgs::msg::Vector3::SharedPtr new_params);

	// ------------------------------ VARIABLES ----------------------------

	// Static variables
    static double mean;                              // Variable for the control time average computation

    // Node parameters
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr xd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tcp_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tcp_twist_sub_;

    std::shared_ptr<rclcpp::PublisherBase> vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wf_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr adm_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr push_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ft_client_;

    // Topics and other parameters
    std::string command_topic_, force_feed_topic_, zero_ft_sensor_topic_, ee_pose_topic_, ee_vel_topic_;
	std::string manipulator_name_, manipulator_;
    double loop_rate_;
    int    n_joints_;
    std::vector<std::string> joint_names_;
    
    // Controller instance
    std::shared_ptr<AdmittanceController> adm_controller_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_adm_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr b_adm_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr k_adm_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_adm_rot_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr b_adm_rot_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr k_adm_rot_sub_;

    
    // KDL instance (for mode "kdl")
    std::shared_ptr<ManipulatorKDL> robot_kdl_;

    // Wrench filter
    std::shared_ptr<filters::RCFilter> force_filter_;
    
    // Matrices and vectors for control
    Eigen::VectorXd q_, xd_, dx_, dx_des_, ddx_des_, wrench_, ee_pose_;
    Eigen::VectorXd force_limit_;
	double kp_pos_, kp_rot_, dz_force_, dz_torque_;
    
	// Mode selection flag
    bool mode_bool_;
    std::string mode_;
};

#endif // ADMITTANCE_CONTROL_H
