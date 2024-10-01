#include "admittance_control/admittance_control.h"

AdmittanceControl::AdmittanceControl()
{
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AdmittanceControl::jointCallback, this);
    force_sub_ = nh_.subscribe("/ur_rtde/ft_sensor", 1, &AdmittanceControl::forceSensorCallback, this);

    joint_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command", 1, true);

    adm_service_ = nh_.advertiseService("/enable_admittance", &AdmittanceControl::enableAdmittance, this);
    ft_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_rtde/zeroFTSensor");

    // joint_position_.setZero();
    wrench_.setZero();
    // ddx_.setZero();
    // dx_.setZero();
    // x_.setZero();
    // M_des_.setZero();
    // B_des_.setZero();
    // K_err_.setZero();
    // for (uint i = 0; i < 3; i++)
    // {
    //     K_err_(i, i) = 1.0;
    //     K_err_(i + 3, i + 3) = 1.0;
    // }
    // joint_vel_.data.resize(6);
    // jacobian_.resize(6);

    Eigen::MatrixXd Mdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Bdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Kdes = Eigen::MatrixXd::Zero(6, 6);
    std::string manipulator_name = "ur5e";
    int n_joints = 6;
    double ts = 0.002;
    std::vector<double> mass = {2.5, 2.5, 2.5, 0.05, 0.05, 0.05};
    std::vector<double> damping = {5.0, 5.0, 5.0, 0.3, 0.3, 0.3};

    for (uint i = 0; i < 6; i++)
    {
        Mdes(i, i) = 5.0 * mass[i];
        Bdes(i, i) = 5.0 * damping[i];
    }

    adm_controller_ = new AdmittanceController(Mdes, Kdes, Bdes, manipulator_name, n_joints, ts);
    wrench_.setZero();

    // robot_kdl = new ManipulatorKDL(manipulator_name);

    // ddx_filter_ = new andrea_filters::RCFilter(6, 30, ts_);
    // admittance_active_ = false;
}

void AdmittanceControl::jointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    adm_controller_->updateJoints(js->position);
    //     for (uint i = 0; i < N_JOINTS; i++)
    //         joint_position_(i, 0) = js->position[i];
    //     robot_kdl->jac(js->position, jacobian_);
    //     robot_kdl->fk(js->position, p_real_);
}

void AdmittanceControl::forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w)
{
    wrench_(0, 0) = w->force.x;
    wrench_(1, 0) = w->force.y;
    wrench_(2, 0) = w->force.z;

    wrench_(3, 0) = w->torque.x;
    wrench_(4, 0) = w->torque.y;
    wrench_(5, 0) = w->torque.z;
}

bool AdmittanceControl::enableAdmittance(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    //     admittance_active_ = req.data;
    if (req.data)
    {
        adm_controller_->enableAdmittance();
        std_srvs::Trigger srv;
        ft_client_.call(srv);
        ros::spinOnce();

        //         std::vector<double> q_tmp(N_JOINTS, 0.0);
        //         for (uint i = 0; i < N_JOINTS; i++)
        //             q_tmp[i] = joint_position_(i, 0);
        //         robot_kdl->fk(q_tmp, p_real_);

        //         x_(0, 0) = p_real_.position.x;
        //         x_(1, 0) = p_real_.position.y;
        //         x_(2, 0) = p_real_.position.z;

        //         x_(3, 0) = p_real_.orientation.w;
        //         x_(4, 0) = p_real_.orientation.x;
        //         x_(5, 0) = p_real_.orientation.y;
        //         x_(6, 0) = p_real_.orientation.z;

        //         ddx_filter_->reset(std::vector<double>(6, 0.0));
    }
    else
    {
        adm_controller_->disableAdmittance();
        //     joint_vel_.data = std::vector<double>(N_JOINTS, 0.0);
        //     joint_vel_pub_.publish(joint_vel_);
    }
    res.success = true;

    return true;
}

// double AdmittanceControl::cutSignal(double x, double dead_zone)
// {
//     if (x > dead_zone)
//         return x - dead_zone;
//     else if (x < -dead_zone)
//         return x + dead_zone;
//     else
//         return 0.0;
// }

// void AdmittanceControl::computeDeadSignal(Eigen::Matrix<double, 6, 1> &f, double dead_zone)
// {
//     f = f.unaryExpr([this, dead_zone](double val)
//                     { return this->cutSignal(val, dead_zone); });
// }

// void AdmittanceControl::computeError(geometry_msgs::Pose &preal, geometry_msgs::Pose &pdes, Eigen::Matrix<double, 6, 1> &err)
// {
//     err_.setZero();

//     err_(0, 0) = pdes.position.x - preal.position.x;
//     err_(1, 0) = pdes.position.y - preal.position.y;
//     err_(2, 0) = pdes.position.z - preal.position.z;

//     Eigen::Quaterniond qact(preal.orientation.w, preal.orientation.x, preal.orientation.y, preal.orientation.z);
//     Eigen::Quaterniond qdes(pdes.orientation.w, pdes.orientation.x, pdes.orientation.y, pdes.orientation.z);
//     Eigen::Vector4d tmp_coeff;

//     if (qact.dot(qdes) < 0)
//         qdes.coeffs() = -qdes.coeffs();

//     double theta = std::acos(qact.dot(qdes));

//     Eigen::Quaterniond dq;
//     if (theta == 0)
//         dq = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
//     else
//     {
//         tmp_coeff = theta / std::sin(theta) * (qdes.coeffs() - std::cos(theta) * qact.coeffs());
//         dq = Eigen::Quaterniond(tmp_coeff[3], tmp_coeff[0], tmp_coeff[1], tmp_coeff[2]);
//     }

//     tmp_coeff = 2.0 * (dq * qact.conjugate()).coeffs();
//     err_(3, 0) = tmp_coeff(0);
//     err_(4, 0) = tmp_coeff(1);
//     err_(5, 0) = tmp_coeff(2);
// }

// void AdmittanceControl::exponentialMapQuaternion(Eigen::Quaterniond &q)
// {
//     Eigen::Vector3d v(q.x(), q.y(), q.z());

//     double v_norm = v.norm();

//     if (v_norm < 1e-10)
//     {
//         return;
//     }
//     else
//     {
//         double cos_v_norm = std::cos(v_norm);
//         double sin_v_norm = std::sin(v_norm);

//         Eigen::Vector3d v_unit = v / v_norm;

//         q.w() = std::cos(v_norm);
//         q.vec() = std::sin(v_norm) * v_unit;
//     }
// }

// void AdmittanceControl::computeSpeed()
// {
//     if (!admittance_active_)
//         return;

//     computeDeadSignal(wrench_);

//     ddx_ = M_des_.inverse() * (wrench_ - B_des_ * dx_);
//     std::vector<double> ddx_tmp = std::vector<double>(ddx_.data(), ddx_.data() + ddx_.size());
//     ddx_filter_->filter(ddx_tmp);
//     for (uint i = 0; i < 6; i++)
//         ddx_(i, 0) = ddx_tmp[i];
//     dx_ = dx_ + ddx_ * ts_;

//     p_des_.position.x = x_(0, 0);
//     p_des_.position.y = x_(1, 0);
//     p_des_.position.z = x_(2, 0);

//     p_des_.orientation.w = x_(3, 0);
//     p_des_.orientation.x = x_(4, 0);
//     p_des_.orientation.y = x_(5, 0);
//     p_des_.orientation.z = x_(6, 0);

//     computeError(p_real_, p_des_, err_);

//     for (uint i = 0; i < 6; i++)
//     {
//         for (uint j = 0; j < N_JOINTS; j++)
//         {
//             jacobian_eigen_(i, j) = jacobian_[i][j];
//         }
//     }

//     dq_ = jacobian_eigen_.completeOrthogonalDecomposition().pseudoInverse() * (dx_ + K_err_ * err_);

//     for (uint i = 0; i < 3; i++)
//         x_(i, 0) += dx_(i, 0) * ts_;

//     Eigen::Quaterniond qact(x_(3, 0), x_(4, 0), x_(5, 0), x_(6, 0));
//     Eigen::Quaterniond qw(0.0, dx_(3, 0), dx_(4, 0), dx_(5, 0));
//     qw.coeffs() = 0.5 * ts_ * qw.coeffs();
//     exponentialMapQuaternion(qw);

//     qact = qact * qw;

//     x_(3, 0) = qact.w();
//     x_(4, 0) = qact.x();
//     x_(5, 0) = qact.y();
//     x_(6, 0) = qact.z();

//     for (uint i = 0; i < N_JOINTS; i++)
//         joint_vel_.data[i] = dq_(i, 0);

//     joint_vel_pub_.publish(joint_vel_);
// }

void AdmittanceControl::spinner()
{
    ros::spinOnce();
    auto dq = adm_controller_->computeSpeed(wrench_);
    std_msgs::Float64MultiArray joint_vel;
    for (uint i = 0; i < dq.rows(); i++)
        joint_vel.data.push_back(dq(i, 0));

    joint_vel_pub_.publish(joint_vel);
}
