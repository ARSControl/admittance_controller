#include "admittance_controller/admittance_controller.h"

AdmittanceController::AdmittanceController(Eigen::Matrix<double, 6, 6> Mdes, Eigen::Matrix<double, 6, 6> Kdes, Eigen::Matrix<double, 6, 6> Bdes, std::string name, int n_joints, double ts)
{
    M_des_ = Mdes;
    B_des_ = Bdes;
    manipulator_name_ = name;
    robot_kdl = new ManipulatorKDL(manipulator_name_);

    n_joints_ = n_joints;

    joint_position_.resize(n_joints, 1);
    dq_.resize(n_joints_, 1);
    ddq_.resize(n_joints_, 1);
    jacobian_eigen_.resize(6, n_joints_);

    joint_position_.setZero();
    wrench_.setZero();
    ddx_.setZero();
    dx_.setZero();
    x_.setZero();
    dq_.setZero();
    ddq_.setZero();
    K_int_.setZero();

    K_des_ = Kdes;

    for (uint i = 0; i < 3; i++)
    {
        K_int_(i, i) = 1.0;
        K_int_(i + 3, i + 3) = 1.0;
    }

    ts_ = ts;
    jacobian_.resize(6);

    for (uint i = 0; i < 6; i++)
        jacobian_[i].resize(n_joints_);

    ddx_filter_ = new andrea_filters::RCFilter(6, 30, ts_);
    admittance_active_ = false;
    dead_zone_force_ = 5.0;
    dead_zone_torque_ = 0.5;
}

bool AdmittanceController::changeParameters(Eigen::Matrix<double, 6, 6> Mdes, Eigen::Matrix<double, 6, 6> Bdes)
{
    if (Mdes.rows() + Mdes.cols() + Bdes.rows() + Bdes.cols() != 24)
        return false;
    M_des_ = Mdes;
    B_des_ = Bdes;
}

void AdmittanceController::enableAdmittance()
{
    if (!state_updated_)
    {
        std::cout << "Error: You Should update the state calliing updateJoints()!" << std::endl;
        return;
    }
    dq_.setZero();
    dx_.setZero();
    ddx_filter_->reset(std::vector<double>(6, 0.0));

    x_(0, 0) = p_real_.position.x;
    x_(1, 0) = p_real_.position.y;
    x_(2, 0) = p_real_.position.z;

    x_(3, 0) = p_real_.orientation.w;
    x_(4, 0) = p_real_.orientation.x;
    x_(5, 0) = p_real_.orientation.y;
    x_(6, 0) = p_real_.orientation.z;

    admittance_active_ = true;
}

void AdmittanceController::disableAdmittance()
{
    admittance_active_ = false;
    state_updated_ = false;
    dq_.setZero();
    dx_.setZero();
}

void AdmittanceController::changeInternalP(Eigen::Matrix<double, 6, 6> Kint)
{
    K_int_ = Kint;
}

void AdmittanceController::setDeadZone(double dead_zone_force, double dead_zone_torque)
{
    dead_zone_force_ = dead_zone_force;
    dead_zone_torque_ = dead_zone_torque;
}

void AdmittanceController::setFilterParams(double cutoff)
{
    ddx_filter_ = new andrea_filters::RCFilter(6, cutoff, ts_);
}

double AdmittanceController::cutSignal(double x, double dead_zone)
{
    if (x > dead_zone)
        return x - dead_zone;
    else if (x < -dead_zone)
        return x + dead_zone;
    else
        return 0.0;
}

void AdmittanceController::computeDeadSignal(Eigen::Matrix<double, 6, 1> &f, double dead_zone_force, double dead_zone_torque)
{
    for (uint i = 0; i < 3; i++)
    {
        f(i, 0) = cutSignal(f(i, 0), dead_zone_force);
        f(i + 3, 0) = cutSignal(f(i + 3, 0), dead_zone_torque);
    }
    // f = f.unaryExpr([this, dead_zone](double val)
    //                 { return this->cutSignal(val, dead_zone); });
}

void AdmittanceController::computeError(geometry_msgs::Pose &preal, geometry_msgs::Pose &pdes, Eigen::Matrix<double, 6, 1> &err)
{
    err_.setZero();

    err_(0, 0) = pdes.position.x - preal.position.x;
    err_(1, 0) = pdes.position.y - preal.position.y;
    err_(2, 0) = pdes.position.z - preal.position.z;

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

        q.w() = std::cos(v_norm);
        q.vec() = std::sin(v_norm) * v_unit;
    }
}

void AdmittanceController::updateJoints(std::vector<double> q)
{
    state_updated_ = true;
    robot_kdl->jac(q, jacobian_);
    robot_kdl->fk(q, p_real_);
}

Eigen::MatrixXd AdmittanceController::computeSpeed(Eigen::Matrix<double, 6, 1> wrench)
{
    Eigen::Matrix<double, 6, 1> dx_des = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 6, 1> ddx_des = Eigen::MatrixXd::Zero(6, 1);
    Eigen::Matrix<double, 7, 1> x_des = Eigen::MatrixXd::Zero(7, 1);
    return computeSpeed(wrench, x_des, dx_des, ddx_des);
}

Eigen::MatrixXd AdmittanceController::computeSpeed(Eigen::Matrix<double, 6, 1> wrench, Eigen::Matrix<double, 7, 1> xdes, Eigen::Matrix<double, 6, 1> dx_des, Eigen::Matrix<double, 6, 1> ddx_des)
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

    p_des_.position.x = xdes(0, 0);
    p_des_.position.y = xdes(1, 0);
    p_des_.position.z = xdes(2, 0);

    p_des_.orientation.w = xdes(3, 0);
    p_des_.orientation.x = xdes(4, 0);
    p_des_.orientation.y = xdes(5, 0);
    p_des_.orientation.z = xdes(6, 0);

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
}
