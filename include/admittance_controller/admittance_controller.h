#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include "filters_c/RCFilter.hpp"

class AdmittanceController
{
public:
	AdmittanceController(const double& m_des,    const double& b_des, const double &k_des, 
						 const  int&   n_joints, const double& ts,
						 const double& dz_force, const double& dz_torque,
						 const double& kp_pos,   const double& kp_rot,
						 const double& kp_push,  const double& push_force_goal,
						 const double& safe_push_dist,
						 const double& acc_filter_freq);
	Eigen::VectorXd computeEESpeed( 	  Eigen::VectorXd &wrench,
                                    const Eigen::VectorXd &xd,
                                    const Eigen::VectorXd &ee_pose,
                                    const Eigen::VectorXd &dx,
                                    const Eigen::VectorXd &dx_des,
                                    const Eigen::VectorXd &ddx_des);

	Eigen::VectorXd pushRegulation(const Eigen::VectorXd &wrench,const Eigen::VectorXd &xd,const Eigen::VectorXd &ee_pose);
	Eigen::VectorXd computeError(const Eigen::VectorXd &preal, const Eigen::VectorXd &pdes);

	bool changeParameters(const Eigen::MatrixXd &Mdes,const Eigen::MatrixXd &Bdes,const Eigen::MatrixXd &Kdes);
	void setAdmittanceParam(const unsigned int  &matrix,
                            const unsigned int  &index,
                            const double        &value);

	void changeInternalP(const double &k_pos, const double &k_rot);

	void enableAdmittance();
	void disableAdmittance();
	void enablePush();
	void disablePush();

	void setDeadZone(const double &dead_zone_force, const double &dead_zone_torque);
	void cutSignal(double &x, const double &dead_zone);
	void computeDeadSignal(Eigen::VectorXd &f, const double &dead_zone_force, const double &dead_zone_torque);
	void setFilterParams(const double &cutoff);

	void exponentialMapQuaternion(Eigen::Quaterniond &q);

	void setToZeroIfSmall(double &value);
	double sign(double value);

private:

	// Variables
	int n_joints_;
	double ts_;	

	double kp_push_;
	double push_force_goal_;
	double safe_push_dist_;

	Eigen::MatrixXd M_des_;
	Eigen::MatrixXd B_des_;
	Eigen::MatrixXd K_des_;
	Eigen::MatrixXd K_int_;

	filters::RCFilter *ddx_filter_;

	bool admittance_active_;
	bool pushing_reg_active_;

	double dead_zone_force_;
	double dead_zone_torque_;

	Eigen::VectorXd cumulative_step_;
};

#endif /* ADMITTANCE_CONTROLLER_H */