#ifndef ADMITTANCE_CONTROL_HPP_
#define ADMITTANCE_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <eigen3/Eigen/Dense>
#include <admittance_controller/admittance_controller.h>

class AdmittanceControl : public rclcpp::Node
{
public:
    AdmittanceControl();

    void spinner();

private:

	// -------------------- FUNCTIONS --------------------
    // ----- Callbacks -----
    void enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    void enablePush(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
					
	void admittanceXdCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);
	void forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w);
	void jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
	void tcpPoseCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg);
	void tcpVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

	// ----- Admittance Change Callbacks -----
	void changeMassAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
	void changeDampAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
	void changeStiffAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
	void changeMassRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
	void changeDampRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);
	void changeStiffRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> msg);

	void updateParamOnWrenchError(const Eigen::VectorXd& wrench, const double& reference_force);

	// ----- Utility Functions -----
	void check_params();
	void shutdown_handler();

	Eigen::VectorXd 	wrenchFilter(const Eigen::VectorXd& wrench);
	Eigen::Quaterniond 	quaternion_from_euler(const double& roll, const double& pitch, const double& yaw);
	Eigen::Vector3d 	euler_from_quaternion(const Eigen::Quaterniond& quaternion);

    // ----- Core Control Logic -----
    void computeAdmittanceControl();

	// --------------- VARIABLES ----------------
    // ----- Parameters -----
    std::string manipulator_name_;
    double loop_rate_;
	int n_joints_;
	double push_force_goal_;

	// ------ Global Variables -----
	Eigen::VectorXd wrench_;  // Current wrench from the force-torque sensor
	Eigen::VectorXd ee_pose_; // Current end-effector pose
	Eigen::VectorXd xd_;      // Desired end-effector pose
	Eigen::VectorXd dx_;      // Current end-effector velocity
	Eigen::VectorXd dx_des_;  // Desired end-effector velocity
	Eigen::VectorXd ddx_des_; // Desired end-effector acceleration
	Eigen::VectorXd q_;       // Current joint positions

	// ----- Spinner Variables -----
	double spinner_mean_ = 0.0; 			// Mean time for the spinner loop
	unsigned long long int num_samples = 0; // Number of samples for the mean time calculation

	// ----- Admittance Controller -----
	AdmittanceController* adm_controller_;
	filters::RCFilter *force_filter_;

	// ----- Admittance Parameters -----
	std::vector<double> m_d;
	std::vector<double> b_d;
	std::vector<double> k_d;
	std::vector<double> m_d_block;
	std::vector<double> b_d_block;
	std::vector<double> k_d_block;

    // ----- ROS Interfaces -----
    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr adm_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr push_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ft_client_;
	// Publishers
	rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wf_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_vel_pub_;
	// Subscribers
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr 	joint_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr 	 	ee_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr 	 	xd_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr 	 	tcp_pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr 	 	tcp_vel_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr  	force_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	m_adm_pos_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	b_adm_pos_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	k_adm_pos_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	m_adm_rot_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	b_adm_rot_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr 	k_adm_rot_sub_;

    // Executor and Timer
    rclcpp::executors::MultiThreadedExecutor executor_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // ADMITTANCE_CONTROL_HPP_
