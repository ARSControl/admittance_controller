#ifndef ADMITTANCE_CONTROL_H
#define ADMITTANCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "energy_tank/msg/inertia_damping.hpp"
#include <fstream>

#include <admittance_controller/admittance_controller.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class AdmittanceControl : public rclcpp::Node
{
public:
	AdmittanceControl(const std::string& node_name);
    ~AdmittanceControl();
	void spinner(void);
	void writeToCSV();

private:
    void check_params();
	void jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg);
    void forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w);
    void inertiaDampingCallback(const std::shared_ptr<energy_tank::msg::InertiaDamping> new_params);

    bool enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
		std::shared_ptr<std_srvs::srv::SetBool::Response> res);

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr force_sub_;
	rclcpp::Subscription<energy_tank::msg::InertiaDamping>::SharedPtr var_adm_sub_;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_vel_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cartesian_vel_pub_;

	rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr adm_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ft_client_;

	Eigen::Matrix<double, 6, 1> wrench_;
	std::vector<double> joint_pos_;
    std::shared_ptr<AdmittanceController> adm_controller_;

	rclcpp::Time start_time_;

    std::string joints_state_topic_,        // Joint States topic
                force_feed_topic_,          // Force-feed topic
                new_adm_params_topic_,      // New Adm Params topic
                command_topic_,             // Joint Velocity command topic
                cart_vel_topic_,            // Cartesian Velocity topic
                enable_adm_service_,        // Enable admittance service call
                zero_ft_sensor_topic_;      // ZeroFT client topic

    bool data_logger_enabled_;              // Data logger enabling var

	// Logger
    std::vector<std::tuple< 
      double , 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>, 
      std::optional<double>,
      std::optional<double>
    >> data_;
};

#endif /* ADMITTANCE_CONTROL */