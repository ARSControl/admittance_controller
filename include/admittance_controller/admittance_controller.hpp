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

#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include <eigen3/Eigen/Eigen>
#include <vector>
#include "filters/RCFilter.h"
#include "geometry_msgs/Vector3.h"

class AdmittanceController
{
public:
    // Constructor
    AdmittanceController(const Eigen::Matrix<double, 6, 6> &Mdes,
                         const Eigen::Matrix<double, 6, 6> &Kdes, 
                         const Eigen::Matrix<double, 6, 6> &Bdes,
                         const double& n_joints, const double& ts,
                         const double& dz_force, const double& dz_torque,
                         const double& kp_pos,   const double& kp_rot,
                         const double& kp_push,  const double& push_force_goal,
                         const double& safe_push_dist);

    // Admittance methods
    Eigen::VectorXd computeQSpeed(		Eigen::VectorXd &wrench,
                                  const Eigen::VectorXd &xd,
                                  const Eigen::VectorXd &ee_pose,
                                  const Eigen::VectorXd &dx,
                                  const Eigen::VectorXd &dx_des,
                                  const Eigen::VectorXd &ddx_des,
                                  const Eigen::MatrixXd &jacobian);

    Eigen::VectorXd computeEESpeed(		 Eigen::VectorXd &wrench,
                                   const Eigen::VectorXd &xd,
                                   const Eigen::VectorXd &ee_pose,
                                   const Eigen::VectorXd &dx,
                                   const Eigen::VectorXd &dx_des,
                                   const Eigen::VectorXd &ddx_des);

    // Variable admittance parameters
    bool changeParameters(const Eigen::Matrix<double, 6, 6> &Mdes,
                          const Eigen::Matrix<double, 6, 6> &Bdes,
                          const Eigen::Matrix<double, 6, 6> &Kdes);

    void setAdmittanceParam(const unsigned int  &matrix,
                            const unsigned int  &index,
                            const double        &value);

    // Enable/disable admittance
    void enableAdmittance();
    void disableAdmittance();
    // Enable/disable pushing control
    void enablePush(void);
    void disablePush(void);

private:

    // Force handling methods
    void cutSignal(double &x, const double &dead_zone);
    void computeDeadSignal(Eigen::VectorXd &f, const double &dead_zone_force, const double &dead_zone_torque);
    void setDeadZone(const double &dead_zone_force, const double &dead_zone_torque);
    Eigen::VectorXd pushRegulation(const Eigen::VectorXd &wrench,const Eigen::VectorXd &xd,const Eigen::VectorXd &ee_pose);

    // Math utils
    double sign(double value);
    void setToZeroIfSmall(double &value);

    // Quaternion handling
    void exponentialMapQuaternion(Eigen::Quaterniond &q);

    // Stiffness matrix for integral error
    void changeInternalP(const double &k_pos, const double &k_rot);

    // Filter handling
    void setFilterParams(const double &cutoff);

    // Setpoint computation
    Eigen::VectorXd computeError(const Eigen::VectorXd &preal, const Eigen::VectorXd &pdes);

    // Empty function for future use (computing acceleration)
    Eigen::VectorXd computeAcceleration();

    // Private member variables
    Eigen::Matrix<double, 6, 6> M_des_, B_des_, K_des_;
    Eigen::Matrix<double, 6, 6> K_int_;
    double n_joints_;
    double ts_;
    double dead_zone_force_, dead_zone_torque_;
    bool   admittance_active_;
    bool   pushing_reg_active_;
    filters::RCFilter* ddx_filter_;
    double kp_push_;
    double push_force_goal_;
    double safe_push_dist_;
    Eigen::VectorXd cumulative_step_;
};

#endif // ADMITTANCE_CONTROLLER_H
