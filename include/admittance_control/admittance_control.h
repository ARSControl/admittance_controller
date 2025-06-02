#ifndef ADMITTANCE_CONTROL_H
#define ADMITTANCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <admittance_controller/admittance_controller.h>

class AdmittanceControl : public rclcpp::Node
{
public:
	AdmittanceControl(const std::string& node_name);
	void spinner(void);

private:
    void check_params();
	void jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w);

    bool enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		std::shared_ptr<std_srvs::srv::SetBool::Response> res);

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_vel_pub_;

	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr adm_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ft_client_;

	Eigen::Matrix<double, 6, 1> wrench_;
    std::shared_ptr<AdmittanceController> adm_controller_;
};

#endif /* ADMITTANCE_CONTROL */