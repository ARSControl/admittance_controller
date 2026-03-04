#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "filters_c/RCFilter.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "manipulator_kdl/manipulator_kdl.hpp"

class AdmittanceController
{
public:
	AdmittanceController(Eigen::Matrix<double, 6, 6> Mdes, Eigen::Matrix<double, 6, 6> Kdes, Eigen::Matrix<double, 6, 6> Bdes, std::string name, int n_joints, double ts);
	Eigen::MatrixXd computeSpeed(Eigen::Matrix<double, 6, 1> &wrench, Eigen::Matrix<double, 7, 1> &xdes, Eigen::Matrix<double, 6, 1> &dx_des, Eigen::Matrix<double, 6, 1> &ddx_des);
	Eigen::MatrixXd computeSpeed(Eigen::Matrix<double, 6, 1> &wrench);
	Eigen::Matrix<double, 6, 1> returnTwist();
	void updateJoints(const std::vector<double> &q);
	bool changeParameters(Eigen::Matrix<double, 6, 6> &Mdes, Eigen::Matrix<double, 6, 6> &Bdes);
	void changeInternalP(Eigen::Matrix<double, 6, 6> &Kint);
	void enableAdmittance();
	void disableAdmittance();
	void setDeadZone(double &dead_zone_force, double &dead_zone_torque);
	void setFilterParams(double &cutoff);


private:
	void computeError(geometry_msgs::msg::Pose &preal, geometry_msgs::msg::Pose &pdes, Eigen::Matrix<double, 6, 1> &err);
	Eigen::MatrixXd computeAcceleration();

	double cutSignal(double &x, double &dead_zone);
	void computeDeadSignal(Eigen::Matrix<double, 6, 1> &f, double &dead_zone_force, double &dead_zone_torque);
	void exponentialMapQuaternion(Eigen::Quaterniond &q);

	std::string manipulator_name_;

	Eigen::MatrixXd joint_position_;
	Eigen::Matrix<double, 6, 1> wrench_;
	Eigen::Matrix<double, 6, 1> ddx_;
	Eigen::Matrix<double, 6, 1> dx_;
	Eigen::Matrix<double, 7, 1> x_;
	Eigen::MatrixXd dq_;
	Eigen::MatrixXd ddq_;
	Eigen::Matrix<double, 6, 6> M_des_;
	Eigen::Matrix<double, 6, 6> B_des_;

	Eigen::Matrix<double, 6, 6> K_des_;
	Eigen::Matrix<double, 6, 6> K_int_;
	Eigen::Matrix<double, 6, 1> err_;

	ManipulatorKDL *robot_kdl;
	int n_joints_;
	double ts_;
	std::vector<std::vector<double>> jacobian_;
	Eigen::MatrixXd jacobian_eigen_;
	geometry_msgs::msg::Pose p_real_;
	geometry_msgs::msg::Pose p_des_;

	filters::RCFilter *ddx_filter_;
	bool admittance_active_;
	bool state_updated_;
	double dead_zone_force_;
	double dead_zone_torque_;
};

#endif /* ADMITTANCE_CONTROLLER_H */