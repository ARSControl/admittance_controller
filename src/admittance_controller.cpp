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

#include "admittance_controller/admittance_controller.h"

// Constructor for the AdmittanceController class
AdmittanceController::AdmittanceController(const Eigen::Matrix<double, 6, 6> &Mdes,
                                           const Eigen::Matrix<double, 6, 6> &Kdes, 
                                           const Eigen::Matrix<double, 6, 6> &Bdes,
                                           const double& n_joints, const double& ts,
                                           const double& dz_force, const double& dz_torque,
                                           const double& kp_pos,   const double& kp_rot,
                                           const double& kp_push,  const double& push_force_goal,
                                           const double& safe_push_dist)
{
    // Initialize desired mass, damping, and stiffness matrices
    M_des_ = Mdes;
    B_des_ = Bdes;
    K_des_ = Kdes;

    // Initialize number of joints and time step
    n_joints_   = n_joints;
    ts_         = ts;

    // Initialize internal stiffness matrix to compensate integral error
    changeInternalP(kp_pos,kp_rot);

    // Initialize low-pass filter for acceleration
    ddx_filter_ = new filters::RCFilter(6, 30, ts_);

    // Initialize admittance control parameters
    admittance_active_  = false;
    pushing_reg_active_ = false;
    setDeadZone(dz_force,dz_torque);

    // Inizialize pushing control
    kp_push_         = kp_push;
    push_force_goal_ = push_force_goal;
    safe_push_dist_  = safe_push_dist;
    cumulative_step_ = 0.;
}

// ----------------------------- MATH UTILS  ----------------------------------- //
void AdmittanceController::setToZeroIfSmall(double &value)
{
    if (std::abs(value) < 1e-20) {value = 0.0;}
}

double AdmittanceController::sign(double value)
{
    if      (value < 0) {return -1.;}
    else if (value > 0) {return +1.;}
    else                {return +0.;}
}

// ----------------------------- ADMITTANCE  ----------------------------------- //
Eigen::VectorXd AdmittanceController::pushRegulation(const Eigen::VectorXd &wrench,const Eigen::VectorXd &xd,const Eigen::VectorXd &ee_pose)
{
    Eigen::VectorXd pushed_xd = xd;

    // If the robot is in contact with something over x direction
    if (wrench(0) < -0.01)
    {
        // If the robot has not been moved a lot under the pushing task
        if (std::abs(ee_pose(0) - xd(0)) < safe_push_dist_)
        {
            // Update the goal over the pushing direction
            cumulative_step_ += kp_push_ / K_des_(0,0) * (wrench(0) + push_force_goal_);
        }
    }
    // If the robot is no more in contact, reset the pushing distance adjustement
    else
    {
        cumulative_step_ = 0.;
    }

    // Return the result
    pushed_xd(0) = xd(0) + cumulative_step_;
    return pushed_xd;
}

// Method to compute joints velocities for kdl mode
Eigen::VectorXd AdmittanceController::computeQSpeed(      Eigen::VectorXd &wrench,
                                                    const Eigen::VectorXd &xd,
                                                    const Eigen::VectorXd &ee_pose,
                                                    const Eigen::VectorXd &dx,
                                                    const Eigen::VectorXd &dx_des,
                                                    const Eigen::VectorXd &ddx_des,
                                                    const Eigen::MatrixXd &jacobian)
{
    // Initialize joints velocity vector as zero
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(n_joints_);
    if (!admittance_active_) {return dq;}

    // Compute force considering dead signal
    computeDeadSignal(wrench, dead_zone_force_, dead_zone_torque_);

    // Update xd to adapt to external force over interaction direction, then compute pose error
    Eigen::VectorXd pose_err;
    if (pushing_reg_active_)    {pose_err = computeError(ee_pose, pushRegulation(wrench,xd,ee_pose));}
    else                        {pose_err = computeError(ee_pose, xd);}

    // Compute the acceleration of the system
    Eigen::VectorXd ddx = ddx_des + M_des_.inverse() * (wrench + B_des_ * (dx_des - dx) + K_des_ * pose_err);

    // Apply the low-pass filter to the acceleration
    ddx = ddx_filter_->filter(ddx);

    // Increment the speed setpoint
    Eigen::VectorXd dx_res = dx + ddx * ts_;

    // Compute the speed setpoint to all joints
    dq = jacobian.completeOrthogonalDecomposition().pseudoInverse() * (dx_res + K_int_ * pose_err);

    // Set a minimum speed value
    for (unsigned int k = 0; k < n_joints_; k++) {setToZeroIfSmall(dq[k]);}

    // Return dq result
    return dq;
}

// Method to compute joints velocities for kdl mode
Eigen::VectorXd AdmittanceController::computeEESpeed(     Eigen::VectorXd &wrench,
                                                    const Eigen::VectorXd &xd,
                                                    const Eigen::VectorXd &ee_pose,
                                                    const Eigen::VectorXd &dx,
                                                    const Eigen::VectorXd &dx_des,
                                                    const Eigen::VectorXd &ddx_des)
{
    // Initialize joints velocity vector as zero
    Eigen::VectorXd dx_res = Eigen::VectorXd::Zero(6);
    if (!admittance_active_) {return dx_res;}

    // Compute force considering dead signal (wrench must be eventually alredy filtered)
    computeDeadSignal(wrench, dead_zone_force_, dead_zone_torque_);
    
    // Update xd to adapt to external force over interaction direction, then compute pose error
    Eigen::VectorXd pose_err;
    if (pushing_reg_active_)    {pose_err = computeError(ee_pose, pushRegulation(wrench,xd,ee_pose));}
    else                        {pose_err = computeError(ee_pose, xd);}

    // Compute the acceleration of the system
    Eigen::VectorXd ddx = ddx_des + M_des_.inverse() * (wrench + B_des_ * (dx_des - dx) + K_des_ * pose_err);

    // Apply the low-pass filter to the acceleration
    ddx = ddx_filter_->filter(ddx);

    // Increment the speed setpoint
    dx_res = dx + ddx * ts_;

    // Set a minimum speed value
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(dx_res[k]);}

    // Return dx result
    return dx_res + K_int_ * pose_err;
}

// ------------------------- VARIABLE ADMITTANCE PARAMS --------------------------

// Method to change the desired mass and damping matrices
bool AdmittanceController::changeParameters(const Eigen::Matrix<double, 6, 6> &Mdes,
                                            const Eigen::Matrix<double, 6, 6> &Bdes,
                                            const Eigen::Matrix<double, 6, 6> &Kdes)
{
    // Update the desired mass, stiffnes and damping matrices
    M_des_ = Mdes;
    B_des_ = Bdes;
    K_des_ = Kdes;
    return true;
}

// Change a specific value of the admittance matrices
void AdmittanceController::setAdmittanceParam(  const unsigned int  &matrix,
                                                const unsigned int  &index,
                                                const double        &value)
{
    // Arguments details:
    // matrix: 0 (mass), 1 (damping), 2 (stiffness)
    // index : 0 (tx), 1 (ty), 2 (tz), 3 (rx), 4 (ry), 5 (rz)
    // value : the value to insert in the matrix
    // Example: setAdmittanceParam(1,2,value) -> K_des_(2,2) = value

    switch(matrix)
    {
        case 0:
            {
                M_des_(index,index) = value;
            }
            break;
        case 1:
            {
                B_des_(index,index) = value;
            }
            break;
        case 2:
            {
                K_des_(index,index) = value;
            }
            break;
    }
}

// ------------------------- ADMITTANCE ENABLE/DISABLE --------------------------

// Method to enable admittance control
void AdmittanceController::enableAdmittance()
{
    // Reset velocities and filter
    // dq_.setZero();
    // dx_.setZero();
    Eigen::VectorXd reset_data = Eigen::VectorXd::Zero(6); 
    ddx_filter_->reset(reset_data);

    // Activate admittance control
    admittance_active_ = true;
}

// Method to disable admittance control
void AdmittanceController::disableAdmittance()
{
    // Reset velocities and filter
    // dq_.setZero();
    // dx_.setZero();
    Eigen::VectorXd reset_data = Eigen::VectorXd::Zero(6); 
    ddx_filter_->reset(reset_data);

    // Disable admittance control
    admittance_active_ = false;
}

// Method to enable pushing control
void AdmittanceController::enablePush()
{
    pushing_reg_active_ = true;
}

// Method to disable pushing control
void AdmittanceController::disablePush()
{
    pushing_reg_active_ = false;
}

// ------------------------------ FORCE HANDLING ---------------------------

// Method to apply dead zone to a signal
void AdmittanceController::cutSignal(double &x, const double &dead_zone)
{
    if      (x > +dead_zone)    {x = x - dead_zone;}
    else if (x < -dead_zone)    {x = x + dead_zone;}
    else                        {x = 0.0;}
}

// Method to apply dead zone to force and torque signals
void AdmittanceController::computeDeadSignal(Eigen::VectorXd &f, const double &dead_zone_force, const double &dead_zone_torque)
{
    for (uint i = 0; i < 3; i++)
    {
        cutSignal(f(i),      dead_zone_force);
        cutSignal(f(i + 3),  dead_zone_torque);
    }
}

// Method to set dead zone parameters for force and torque
void AdmittanceController::setDeadZone(const double &dead_zone_force, const double &dead_zone_torque)
{
    dead_zone_force_  = dead_zone_force;
    dead_zone_torque_ = dead_zone_torque;
}

// --------------------------- QUATERNIONS HANDLING ---------------------------

// Method to apply exponential map to a quaternion
void AdmittanceController::exponentialMapQuaternion(Eigen::Quaterniond &q)
{
    Eigen::Vector3d v(q.x(), q.y(), q.z());
    double v_norm = v.norm();

    if (v_norm < 1e-10)
    {
        return;
    }
    else
    {
        Eigen::Vector3d v_unit = v / v_norm;
        q.w()   = std::cos(v_norm);
        q.vec() = std::sin(v_norm) * v_unit;
    }
}

// ---------------------- VARIABLE K FOR INTEGRAL ERROR -----------------------

// Method to change the internal stiffness matrix
void AdmittanceController::changeInternalP(const double &k_pos, const double &k_rot)
{
    K_int_.setZero();

    for (uint i = 0; i < 3; i++)
    {
        K_int_(i, i)         = k_pos;
        K_int_(i + 3, i + 3) = k_rot;
    }
}

// --------------------------- FILTER HANDLING ------------------------------

// Method to set filter parameters
void AdmittanceController::setFilterParams(const double &cutoff)
{
    delete ddx_filter_;
    ddx_filter_ = new filters::RCFilter(6, cutoff, ts_);
}

// --------------------------- SETPOINT COMPUTATION ---------------------------

// Method to compute the error between the real and desired poses
Eigen::VectorXd AdmittanceController::computeError(const Eigen::VectorXd &preal, const Eigen::VectorXd &pdes)
{
    Eigen::VectorXd err = Eigen::VectorXd::Zero(6);

    // Compute position error
    err(0) = pdes(0) - preal(0);
    err(1) = pdes(1) - preal(1);
    err(2) = pdes(2) - preal(2);

    // Compute orientation error using quaternions
    Eigen::Quaterniond qact(preal(6), preal(3), preal(4), preal(5));
    Eigen::Quaterniond qdes( pdes(6),  pdes(3),  pdes(4),  pdes(5));
    Eigen::Vector4d tmp_coeff;

    if (qact.dot(qdes) < 0) {qdes.coeffs() = -qdes.coeffs();}

    float theta = std::acos(qact.dot(qdes));

    if (theta != theta)
    {
        theta = 0.0;
    }        

    Eigen::Quaterniond dq;
    if (theta == 0.0)
    {
        dq = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    }
    else
    {
        tmp_coeff = theta / std::sin(theta) * (qdes.coeffs() - std::cos(theta) * qact.coeffs());
        dq = Eigen::Quaterniond(tmp_coeff[3], tmp_coeff[0], tmp_coeff[1], tmp_coeff[2]);
    }

    tmp_coeff = 2.0 * (dq * qact.conjugate()).coeffs();

    // Fill the msg to return
    err(3) = tmp_coeff(0);
    err(4) = tmp_coeff(1);
    err(5) = tmp_coeff(2);

    return err;
}

// Empty private function to compute acceleration, still not implemented
Eigen::VectorXd AdmittanceController::computeAcceleration()
{
    // Maybe if implementing the admittance control on a torque controlled robot will be needed
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(6);
    return vector;
}