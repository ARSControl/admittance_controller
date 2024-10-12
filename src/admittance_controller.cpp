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

// Constructor: initializes controller parameters from YAML or defaults
AdmittanceController::AdmittanceController()
{
    // Load diagonal parameters for mass, spring, and damping
    double m_d, k_d, b_d;
    if (!nh.getParam("/admittance_controller/m_d", m_d))
    {
        ROS_WARN("Diagonal mass 'm_d' not set, using default value of 1.0.");
        m_d = 1.0; // Default
    }
    if (!nh.getParam("/admittance_controller/k_d", k_d))
    {
        ROS_WARN("Diagonal spring constant 'k_d' not set, using default value of 100.0.");
        k_d = 100.0; // Default
    }
    if (!nh.getParam("/admittance_controller/b_d", b_d))
    {
        ROS_WARN("Diagonal damping constant 'b_d' not set, using default value of 10.0.");
        b_d = 10.0; // Default
    }

    // Initialize matrices with the diagonal values
    M_des_ = Eigen::MatrixXd::Identity(6, 6) * m_d;
    K_des_ = Eigen::MatrixXd::Identity(6, 6) * k_d;
    B_des_ = Eigen::MatrixXd::Identity(6, 6) * b_d;

    // Load additional parameters
    if (!nh.getParam("/admittance_controller/loop_rate", loop_rate_))
    {
        ROS_WARN("Loop rate not set, using default: 500 Hz.");
        loop_rate_ = 500.0; // Default 500 Hz
    }

    if (!nh.getParam("/admittance_controller/force_limit", force_limit_))
    {
        ROS_WARN("Force limit not set, using default values.");
        force_limit_ = Eigen::VectorXd::Constant(6, 100.0); // Default limits
    }

    if (!nh.getParam("/admittance_controller/manipulator_name", manipulator_name_))
    {
        ROS_WARN("Manipulator name not set, using default: ur5.");
        manipulator_name_ = "ur5"; // Default
    }

    if (!nh.getParam("/admittance_controller/joint_names", joint_names_))
    {
        ROS_WARN("Joint names not set, using default names.");
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}; // Default UR5 joints
    }

    n_joints_ = joint_names_.size();

    // Additional initialization logic
}

void AdmittanceController::publishControlMode(const std::string& mode)
{
    if (mode == "moveit")
    {
        // Publish Cartesian velocity to manipulator_name/cmd_vel
        // Logic for MoveIt mode
    }
    else if (mode == "kdl")
    {
        // Convert to joint velocities and publish to joint velocity controller
        // Logic for KDL mode
    }
    else
    {
        ROS_ERROR("Unknown control mode: %s", mode.c_str());
    }
}

// Additional methods implementation...









// VECCHIO CODICE

#include "admittance_controller/admittance_controller.h"

// Constructor for the AdmittanceController class
AdmittanceController::AdmittanceController(Eigen::Matrix<double, n_joints, n_joints> Mdes, Eigen::Matrix<double, 6, 6> Kdes, Eigen::Matrix<double, 6, 6> Bdes,
                                           std::string manipulator_name, int n_joints, double ts)
{
    // Initialize desired mass, damping, and stiffness matrices
    M_des_ = Mdes;
    B_des_ = Bdes;
    K_des_ = Kdes;

    // Initialize manipulator name and KDL model
    manipulator_name_ = manipulator_name;
    robot_kdl = new ManipulatorKDL(manipulator_name_);

    // Initialize number of joints and time step
    n_joints_ = n_joints;
    ts_ = ts;

    // Resize and initialize joint position, velocity, and acceleration vectors
    joint_position_.resize(n_joints, 1);
    dq_.resize(n_joints_, 1);
    ddq_.resize(n_joints_, 1);
    jacobian_eigen_.resize(6, n_joints_);

    // Set initial values to zero
    joint_position_.setZero();
    wrench_.setZero();
    ddx_.setZero();
    dx_.setZero();
    x_.setZero();
    dq_.setZero();
    ddq_.setZero();
    K_int_.setZero();

    // Initialize internal stiffness matrix to compensate integration error
    for (uint i = 0; i < 3; i++)
    {
        K_int_(i, i) = 1.0;
        K_int_(i + 3, i + 3) = 1.0;
    }

    // Resize Jacobian matrix
    jacobian_.resize(6);
    for (uint i = 0; i < 6; i++)
        jacobian_[i].resize(n_joints_);

    // Initialize low-pass filter for acceleration
    ddx_filter_ = new filters::RCFilter(6, 30, ts_);

    // Initialize admittance control parameters
    admittance_active_ = false;
    dead_zone_force_ = 5.0;
    dead_zone_torque_ = 0.5;
}

// Method to change the desired mass and damping matrices
bool AdmittanceController::changeParameters(Eigen::Matrix<double, 6, 6> &Mdes, Eigen::Matrix<double, 6, 6> &Bdes)
{
    // Check if the matrices are of correct size
    if (Mdes.rows() + Mdes.cols() + Bdes.rows() + Bdes.cols() != 24)
        return false;

    // Update the desired mass and damping matrices
    M_des_ = Mdes;
    B_des_ = Bdes;
    return true;
}

// Method to enable admittance control
void AdmittanceController::enableAdmittance()
{
    // Check if the state has been updated
    if (!state_updated_)
    {
        std::cout << "Error: You should update the state by calling updateJoints()!" << std::endl;
        return;
    }

    // Reset velocities and filter
    dq_.setZero();
    dx_.setZero();
    ddx_filter_->reset(std::vector<double>(6, 0.0));

    // Set initial position and orientation
    x_(0, 0) = p_real_.position.x;
    x_(1, 0) = p_real_.position.y;
    x_(2, 0) = p_real_.position.z;
    x_(3, 0) = p_real_.orientation.w;
    x_(4, 0) = p_real_.orientation.x;
    x_(5, 0) = p_real_.orientation.y;
    x_(6, 0) = p_real_.orientation.z;

    // Activate admittance control
    admittance_active_ = true;
}

// Method to disable admittance control
void AdmittanceController::disableAdmittance()
{
    admittance_active_ = false;
    state_updated_ = false;
    dq_.setZero();
    dx_.setZero();
}

// Method to change the internal stiffness matrix
void AdmittanceController::changeInternalP(Eigen::Matrix<double, 6, 6> &Kint)
{
    K_int_ = Kint;
}

// Method to set dead zone parameters for force and torque
void AdmittanceController::setDeadZone(double &dead_zone_force, double &dead_zone_torque)
{
    dead_zone_force_ = dead_zone_force;
    dead_zone_torque_ = dead_zone_torque;
}

// Method to set filter parameters
void AdmittanceController::setFilterParams(double &cutoff)
{
    ddx_filter_ = new filters::RCFilter(6, cutoff, ts_);
}

// Method to apply dead zone to a signal
double AdmittanceController::cutSignal(double &x, double &dead_zone)
{
    if (x > dead_zone)
        return x - dead_zone;
    else if (x < -dead_zone)
        return x + dead_zone;
    else
        return 0.0;
}

// Method to apply dead zone to force and torque signals
void AdmittanceController::computeDeadSignal(Eigen::Matrix<double, 6, 1> &f, double &dead_zone_force, double &dead_zone_torque)
{
    for (uint i = 0; i < 3; i++)
    {
        f(i, 0) = cutSignal(f(i, 0), dead_zone_force);
        f(i + 3, 0) = cutSignal(f(i + 3, 0), dead_zone_torque);
    }
}

// Method to compute the error between the real and desired poses
void AdmittanceController::computeError(geometry_msgs::Pose &preal, geometry_msgs::Pose &pdes, Eigen::Matrix<double, 6, 1> &err)
{
    err_.setZero();

    // Compute position error
    err_(0, 0) = pdes.position.x - preal.position.x;
    err_(1, 0) = pdes.position.y - preal.position.y;
    err_(2, 0) = pdes.position.z - preal.position.z;

    // Compute orientation error using quaternions
    Eigen::Quaterniond qact(preal.orientation.w, preal.orientation.x, preal.orientation.y, preal.orientation.z);
    Eigen::Quaterniond qdes(pdes.orientation.w, pdes.orientation.x, pdes.orientation.y, pdes.orientation.z);
    Eigen::Vector4d tmp_coeff;

    if (qact.dot(qdes) < 0)
        qdes.coeffs() = -qdes.coeffs();

    double theta = std::acos(qact.dot(qdes));

    Eigen::Quaterniond dq;
    if (theta == 0)
        dq = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
    else
    {
        tmp_coeff = theta / std::sin(theta) * (qdes.coeffs() - std::cos(theta) * qact.coeffs());
        dq = Eigen::Quaterniond(tmp_coeff[3], tmp_coeff[0], tmp_coeff[1], tmp_coeff[2]);
    }

    tmp_coeff = 2.0 * (dq * qact.conjugate()).coeffs();
    err_(3, 0) = tmp_coeff(0);
    err_(4, 0) = tmp_coeff(1);
    err_(5, 0) = tmp_coeff(2);
}

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
        double cos_v_norm = std::cos(v_norm);
        double sin_v_norm = std::sin(v_norm);
        Eigen::Vector3d v_unit = v / v_norm;

        q.w() = cos_v_norm;
        q.vec() = sin_v_norm * v_unit;
    }
}

// Method to update joint states
void AdmittanceController::updateJoints(const std::vector<double> &q)
{
    state_updated_ = true;
    robot_kdl->jac(q, jacobian_);
    robot_kdl->fk(q, p_real_);
}

// Method to compute joint velocities based on wrench input
Eigen::MatrixXd AdmittanceController::computeSpeed(Eigen::Matrix<double, 6, 1> &wrench)
{
    Eigen::Matrix<double, 6, 1> dx_des = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> ddx_des = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 7, 1> x_des = Eigen::MatrixXd::Zero(7, 1);
    return computeSpeed(wrench, x_des, dx_des, ddx_des);
}

// Overloaded method to compute joint velocities based on wrench, desired pose, and desired velocities
Eigen::MatrixXd AdmittanceController::computeSpeed(Eigen::Matrix<double, 6, 1> &wrench, Eigen::Matrix<double, 7, 1> &xdes, Eigen::Matrix<double, 6, 1> &dx_des, Eigen::Matrix<double, 6, 1> &ddx_des)
{
    if (!admittance_active_)
        return dq_;

    wrench_ = wrench;
    computeDeadSignal(wrench_, dead_zone_force_, dead_zone_torque_);

    p_des_.position.x = xdes(0, 0);
    p_des_.position.y = xdes(1, 0);
    p_des_.position.z = xdes(2, 0);

    p_des_.orientation.w = xdes(3, 0);
    p_des_.orientation.x = xdes(4, 0);
    p_des_.orientation.y = xdes(5, 0);
    p_des_.orientation.z = xdes(6, 0);

    computeError(p_real_, p_des_, err_);

    ddx_ = ddx_des + M_des_.inverse() * (wrench_ + B_des_ * (dx_des - dx_) + K_des_ * err_);
    std::vector<double>
        ddx_tmp = std::vector<double>(ddx_.data(), ddx_.data() + ddx_.size());
    ddx_filter_->filter(ddx_tmp);
    for (uint i = 0; i < 6; i++)
        ddx_(i, 0) = ddx_tmp[i];
    dx_ = dx_ + ddx_ * ts_;

    for (uint i = 0; i < 6; i++)
    {
        for (uint j = 0; j < n_joints_; j++)
        {
            jacobian_eigen_(i, j) = jacobian_[i][j];
        }
    }

    p_des_.position.x = x_(0, 0);
    p_des_.position.y = x_(1, 0);
    p_des_.position.z = x_(2, 0);

    p_des_.orientation.w = x_(3, 0);
    p_des_.orientation.x = x_(4, 0);
    p_des_.orientation.y = x_(5, 0);
    p_des_.orientation.z = x_(6, 0);

    computeError(p_real_, p_des_, err_);

    dq_ = jacobian_eigen_.completeOrthogonalDecomposition().pseudoInverse() * (dx_ + K_int_ * err_);

    for (uint i = 0; i < 3; i++)
        x_(i, 0) += dx_(i, 0) * ts_;

    Eigen::Quaterniond qact(x_(3, 0), x_(4, 0), x_(5, 0), x_(6, 0));
    Eigen::Quaterniond qw(0.0, dx_(3, 0), dx_(4, 0), dx_(5, 0));
    qw.coeffs() = 0.5 * ts_ * qw.coeffs();
    exponentialMapQuaternion(qw);

    qact = qact * qw;

    x_(3, 0) = qact.w();
    x_(4, 0) = qact.x();
    x_(5, 0) = qact.y();
    x_(6, 0) = qact.z();

    return dq_;
}

Eigen::MatrixXd AdmittanceController::computeAcceleration()
{
    // I do not know if it make sense to compute ddq in another function
    // Maybe if we want to implement admittance control on a torque controlled robot?
    // For know this function is empty and it is private
    Eigen::MatrixXd matrix;
    return matrix;
}
