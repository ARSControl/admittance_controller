#include "admittance_controller/admittance_controller.h"
#include <algorithm>

// Constructor for the AdmittanceController class
AdmittanceController::AdmittanceController( const std::vector<double>& m_des,
                                            const std::vector<double>& k_des,
                                            const std::vector<double>& b_des, 
                                            const int&    n_joints, const double& ts,
                                            const double& dz_force, const double& dz_torque,
                                            const double& kp_pos,   const double& kp_rot,
                                            const double& kp_push,  const double& push_force_goal,
                                            const double& safe_push_dist,
                                            const double& acc_filter_freq)
                                            : n_joints_(n_joints), ts_(ts), 
                                              kp_push_(kp_push), push_force_goal_(push_force_goal),
                                              safe_push_dist_(safe_push_dist),
                                              max_acc_(1e9)
{
    // Initialize number of joints and time step
    n_joints_ = n_joints;
    ts_ = ts;

    // Initialize desired mass, damping, and stiffness matrices to identity
    M_des_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    B_des_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_);
    K_des_ = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

    for (int i = 0; i < n_joints_; i++)
    {
        // Fill the diagonal of the matrices with the provided values
        M_des_(i, i) = m_des[i];
        K_des_(i, i) = k_des[i];
        B_des_(i, i) = b_des[i];
    }

    // Initialize internal stiffness matrix to compensate integral error
    K_int_ = Eigen::MatrixXd::Zero(6, 6);
    changeInternalP(kp_pos,kp_rot);

    // Initialize low-pass filter for acceleration
    ddx_filter_ = new filters::RCFilter(n_joints_, acc_filter_freq, ts_);

    // Initialize admittance control parameters
    admittance_active_  = false;
    pushing_reg_active_ = false;
    setDeadZone(dz_force,dz_torque);

    // Inizialize pushing control
    cumulative_step_ = Eigen::VectorXd::Zero(3);
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
    double push_force_goal_local = push_force_goal_;

    // Iterate over the pushing direction
    for(unsigned int k = 0; k < 1; k++)
    {
        // If the wrench is negative, the cumulative step is positive, meaning the robot is being pushed
        if (wrench(k) < 0)
        {
            push_force_goal_local = -push_force_goal_local;
        }

        // If the robot is in contact with something
        if (std::abs(wrench(k)) > 0.01)
        {
            // If the robot has not been moved a lot under the pushing task
            if (std::abs(ee_pose(k) - xd(k)) < safe_push_dist_)
            {
                // Update the goal over the pushing direction
                cumulative_step_(k) += kp_push_ / K_des_(k,k) * (wrench(k) - push_force_goal_local);
            }
        }
        // If the robot is no more in contact, reset the pushing distance adjustement
        else
        {
            cumulative_step_(k) = 0.;
        }
        // Return the result
        pushed_xd(k) += cumulative_step_(k);
    }

    return pushed_xd;
}

// Method to compute joints velocities for kdl mode
Eigen::VectorXd AdmittanceController::computeEESpeed(      Eigen::VectorXd &wrench,
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

    Eigen::VectorXd ddx = ddx_des + M_des_.inverse() * (wrench + B_des_ * (dx_des - dx) + K_des_ * pose_err);

    // Apply the low-pass filter to the acceleration
    ddx = ddx_filter_->filter(ddx);

    // Clip acceleration command component-wise.
    if (max_acc_ > 0.0) {
        for (int k = 0; k < ddx.size(); ++k) {
            ddx(k) = std::max(-max_acc_, std::min(max_acc_, ddx(k)));
        }
    }

    // Increment the speed setpoint
    dx_res = dx + ddx * ts_;

    // Set a minimum speed value
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(dx_res[k]);}

    // Return dx result
    return dx_res + K_int_ * pose_err;
}

// ------------------------- VARIABLE ADMITTANCE PARAMS --------------------------

// Method to change the desired mass and damping matrices
bool AdmittanceController::changeParameters(const Eigen::MatrixXd &Mdes,
                                            const Eigen::MatrixXd &Bdes,
                                            const Eigen::MatrixXd &Kdes)
{
    // Check for size mismatch
    if (Mdes.rows() != M_des_.rows() || Mdes.cols() != M_des_.cols() ||
        Bdes.rows() != B_des_.rows() || Bdes.cols() != B_des_.cols() ||
        Kdes.rows() != K_des_.rows() || Kdes.cols() != K_des_.cols()  )
    {
        RCLCPP_ERROR(rclcpp::get_logger("AdmittanceController"),
                     "Matrix size mismatch: expected sizes are M: %ldx%ld, B: %ldx%ld, K: %ldx%ld, but got M: %ldx%ld, B: %ldx%ld, K: %ldx%ld",
                     M_des_.rows(), M_des_.cols(),
                     B_des_.rows(), B_des_.cols(),
                     K_des_.rows(), K_des_.cols(),
                     Mdes.rows(),   Mdes.cols(),
                     Bdes.rows(),   Bdes.cols(),
                     Kdes.rows(),   Kdes.cols());
        return false;
    }

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
    // matrix:  0 (mass), 1 (damping), 2 (stiffness)
    // index :  0 (tx), 1 (ty), 2 (tz), 3 (rx), 4 (ry), 5 (rz)
    // value :  the value to insert in the matrix
    // Example: setAdmittanceParam(1,2,value) -> K_des_(2,2) = value

    switch(matrix)
    {
        case 0:
            {
                M_des_(index, index) = value;
            }
            break;
        case 1:
            {
                B_des_(index, index) = value;
            }
            break;
        case 2:
            {
                K_des_(index, index) = value;
            }
            break;
    }
}

// ------------------------- ADMITTANCE ENABLE/DISABLE --------------------------

// Method to enable admittance control
void AdmittanceController::enableAdmittance()
{
    // Reset velocities and filter
    Eigen::VectorXd reset_data = Eigen::VectorXd::Zero(6); 
    ddx_filter_->reset(reset_data);

    // Activate admittance control
    admittance_active_ = true;
}

// Method to disable admittance control
void AdmittanceController::disableAdmittance()
{
    // Reset velocities and filter
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

void AdmittanceController::setPushForceGoal(const double &push_force_goal)
{
    push_force_goal_ = push_force_goal;
}

void AdmittanceController::setMaxAcceleration(const double &max_acc)
{
    max_acc_ = std::max(0.0, max_acc);
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

    // Check if the quaternions are valid
    float theta = std::acos(qact.dot(qdes));

    if (theta != theta)
    {
        theta = 0.0;
    }        

    // Compute the quaternion error
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
