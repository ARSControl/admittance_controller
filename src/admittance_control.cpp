#include "admittance_control/admittance_control.h"

AdmittanceControl::AdmittanceControl()
{
    joint_sub_ = nh_.subscribe("/joint_states", 1, &AdmittanceControl::jointCallback, this);
    force_sub_ = nh_.subscribe("/ur_rtde/ft_sensor", 1, &AdmittanceControl::forceSensorCallback, this);

    joint_vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command", 1, true);

    adm_service_ = nh_.advertiseService("/enable_admittance", &AdmittanceControl::enableAdmittance, this);
    ft_client_ = nh_.serviceClient<std_srvs::Trigger>("/ur_rtde/zeroFTSensor");

    wrench_.setZero();

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
}

void AdmittanceControl::jointCallback(const sensor_msgs::JointState::ConstPtr &js)
{
    adm_controller_->updateJoints(js->position);
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
    if (req.data)
    {
        adm_controller_->enableAdmittance();
        std_srvs::Trigger srv;
        ft_client_.call(srv);
        ros::spinOnce();
    }
    else
    {
        adm_controller_->disableAdmittance();
    }
    res.success = true;

    return true;
}

void AdmittanceControl::spinner()
{
    ros::spinOnce();
    auto dq = adm_controller_->computeSpeed(wrench_);
    std_msgs::Float64MultiArray joint_vel;
    for (uint i = 0; i < dq.rows(); i++)
        joint_vel.data.push_back(dq(i, 0));

    joint_vel_pub_.publish(joint_vel);
}
