














// VECCHIO CODICE



#ifndef ADMITTANCE_CONTROL_H
#define ADMITTANCE_CONTROL_H

#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <admittance_controller/admittance_controller.h>

class AdmittanceControl
{
public:
	// ---------------------- PUBLIC FUNCTIONS -------------------------- //
	AdmittanceControl();	// Constructor
	void spinner(void);		// Code ROS spinner

private:

	// ---------------------- PRIVATE FUNCTIONS -------------------------- //
		void jointCallback(const sensor_msgs::JointState::ConstPtr &js);
		void forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w);
		bool enableAdmittance(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

		// Main function of the computation of the admittance control
		void admittance_control_main(void);

		// Shutdown handler
		static void shutdown_handler(int sig);

	// ---------------------- PRIVATE VARIABLES -------------------------- //
		ros::NodeHandle nh_;
		ros::Subscriber joint_sub_;
		ros::Subscriber force_sub_;
		ros::Publisher joint_vel_pub_;

		ros::ServiceServer adm_service_;
		ros::ServiceClient ft_client_;

		Eigen::Matrix<double, 6, 1> wrench_;
		AdmittanceController *adm_controller_;

		static double mean;                     // Average value for the duration of the admittance control computation
};

#endif /* ADMITTANCE_CONTROL */