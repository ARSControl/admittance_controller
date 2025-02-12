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

#include "admittance_control/admittance_control.hpp"

// --------------------- STATIC VARIABLES ---------------------
double AdmittanceControl::mean = 0.0;  // variable for the control time average computation

// --------------------- PUBLIC CONSTRUCTOR ---------------------
AdmittanceControl::AdmittanceControl(const std::string& node_name) : rclcpp::Node(node_name)
{
    // Initialize wrench to zero
    this->wrench_      = Eigen::VectorXd::Zero(6);
    this->force_limit_ = Eigen::VectorXd::Zero(6);

    // Initialize state vectors
    this->ee_pose_ = Eigen::VectorXd::Zero(7);    // 7 for position + quaternion
    this->xd_      = Eigen::VectorXd::Zero(7);    // 7 for position + quaternion
    this->dx_      = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity
    this->dx_des_  = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity
    this->ddx_des_ = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity

    // Update node params
    check_params();
    RCLCPP_INFO(this->get_logger(), "Params and attributes for admittance control are correctly initialized.");

    // Subscribe to topics
    this->joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&AdmittanceControl::jointCallback, this, std::placeholders::_1));
    this->force_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        this->force_feed_topic_, 1, std::bind(&AdmittanceControl::forceSensorCallback, this, std::placeholders::_1));
    this->xd_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        this->manipulator_name_ + "/adm_xd", 1, std::bind(&AdmittanceControl::admittanceXdCallback, this, std::placeholders::_1));

    
    // Subscribers to change admittance
    this->m_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/m_adm_pos", 1, std::bind(&AdmittanceControl::changeMassAdmittanceCallback, this, std::placeholders::_1));
    this->b_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/b_adm_pos", 1, std::bind(&AdmittanceControl::changeDampAdmittanceCallback, this, std::placeholders::_1));
    this->k_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/k_adm_pos", 1, std::bind(&AdmittanceControl::changeStiffAdmittanceCallback, this, std::placeholders::_1));
    this->m_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/m_adm_rot", 1, std::bind(&AdmittanceControl::changeMassRotAdmittanceCallback, this, std::placeholders::_1));
    this->b_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/b_adm_rot", 1, std::bind(&AdmittanceControl::changeDampRotAdmittanceCallback, this, std::placeholders::_1));
    this->k_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        this->manipulator_name_ + "/k_adm_rot", 1, std::bind(&AdmittanceControl::changeStiffRotAdmittanceCallback, this, std::placeholders::_1));


    // Load and apply parameters
    if (this->mode_ == "kdl") {
        this->vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(this->command_topic_, 1);
    } else {
        this->vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(this->command_topic_, 1);
    }

    this->tcp_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        this->ee_pose_topic_, 1, std::bind(&AdmittanceControl::tcpPoseCallback, this, std::placeholders::_1));
    this->tcp_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        this->ee_vel_topic_, 1, std::bind(&AdmittanceControl::tcpTwistCallback, this, std::placeholders::_1));


    // Advertise service to enable admittance control
    this->adm_service_ = this->create_service<std_srvs::srv::SetBool>(
        this->manipulator_name_ + "/enable_admittance", std::bind(&AdmittanceControl::enableAdmittance, this, std::placeholders::_1, std::placeholders::_2));
    this->push_service_ = this->create_service<std_srvs::srv::SetBool>(
        this->manipulator_name_ + "/enable_push_regulation", std::bind(&AdmittanceControl::enablePushRegulation, this, std::placeholders::_1, std::placeholders::_2));

    // Create service client to zero the force-torque sensor
    ft_client_ = this->create_client<std_srvs::srv::Trigger>(this->zero_ft_sensor_topic_);

    // Create a publisher to show the filtered msh
    wf_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(manipulator_name_ + "/filtered_wrench", 1);

    // Initialize low-pass filter for the wrench
    force_filter_ = std::make_shared<filters::RCFilter>(6, this->force_cut_freq_, 1 / this->loop_rate_);
}

// Node params update
void AdmittanceControl::check_params()
{
    // Init joints
    this->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
    if (!this->has_parameter("joint_names")) {
        RCLCPP_WARN(this->get_logger(), "Joint names not set, using default names.");
        this->declare_parameter<std::vector<std::string>>("joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
    }
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    n_joints_ = joint_names_.size();
    q_ = Eigen::VectorXd::Zero(n_joints_);

    // Init motion params
    this->declare_parameter("m_d", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("k_d", rclcpp::PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter("b_d", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (!this->has_parameter("m_d")) {
        RCLCPP_WARN(this->get_logger(), "Diagonal mass 'm_d' not set, using default value of 1.0.");
        this->declare_parameter<std::vector<double>>("m_d", {1., 1., 1., 1., 1., 1.});
    }
    if (!this->has_parameter("k_d")) {
        RCLCPP_WARN(this->get_logger(), "Diagonal spring constant 'k_d' not set, using default value of 100.0.");
        this->declare_parameter<std::vector<double>>("k_d", {100., 100., 100., 100., 100., 100.});
    }
    if (!this->has_parameter("b_d")) {
        RCLCPP_WARN(this->get_logger(), "Diagonal damping constant 'b_d' not set, using default value of 10.0.");
        this->declare_parameter<std::vector<double>>("b_d", {10., 10., 10., 10., 10., 10.});
    }
    std::vector<double> m_d = this->get_parameter("m_d").as_double_array();
    std::vector<double> k_d = this->get_parameter("k_d").as_double_array();
    std::vector<double> b_d = this->get_parameter("b_d").as_double_array();

    // Admittance params
    this->declare_parameter("dz_force", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("dz_torque", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("kp_pos", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("kp_rot", rclcpp::PARAMETER_DOUBLE);
    if (!this->has_parameter("dz_force")) {
        RCLCPP_WARN(this->get_logger(), "Force dead zone not set, using default: 5 N.");
        this->declare_parameter<double>("dz_force", 5.0);
    }
    if (!this->has_parameter("dz_torque")) {
        RCLCPP_WARN(this->get_logger(), "Torque dead zone not set, using default: 1 N.");
        this->declare_parameter<double>("dz_torque", 1.0);
    }
    if (!this->has_parameter("kp_pos")) {
        RCLCPP_WARN(this->get_logger(), "Proportional gain for position compensation not set, using default: 1.0.");
        this->declare_parameter<double>("kp_pos", 1.0);
    }
    if (!this->has_parameter("kp_rot")) {
        RCLCPP_WARN(this->get_logger(), "Proportional gain for rotation compensation not set, using default: 1.0.");
        this->declare_parameter<double>("kp_rot", 1.0);
    }
    dz_force_  = this->get_parameter("dz_force").as_double();
    dz_torque_ = this->get_parameter("dz_torque").as_double();
    kp_pos_    = this->get_parameter("kp_pos").as_double();
    kp_rot_    = this->get_parameter("kp_rot").as_double();

    // Init interaction params
    this->declare_parameter("force_limit", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (!this->has_parameter("force_limit")) {
        RCLCPP_WARN(this->get_logger(), "Force limit not set, using default values.");
        this->declare_parameter<std::vector<double>>("force_limit", {1.0, 1.0, 1.0, 0.5, 0.5, 0.5});
    }
    std::vector<double> force_limit_vec = this->get_parameter("force_limit").as_double_array();
    for (unsigned int k = 0; k < 6; k++) {
        force_limit_(k) = force_limit_vec[k];
    }
    this->declare_parameter("mode", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("mode")) {
        RCLCPP_WARN(this->get_logger(), "Mode param not set, using default: kdl.");
        this->declare_parameter<std::string>("mode", "kdl");
    }
    mode_ = this->get_parameter("mode").as_string();
    mode_bool_ = (mode_ != "kdl");

    // Init control params
    this->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
    if (!this->has_parameter("loop_rate")) {
        RCLCPP_WARN(this->get_logger(), "Loop rate not set, using default: 500 Hz.");
        this->declare_parameter<double>("loop_rate", 500.0);
    }
    loop_rate_ = this->get_parameter("loop_rate").as_double();

    // Init model params
    this->declare_parameter("manipulator", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("manipulator")) {
        RCLCPP_WARN(this->get_logger(), "Manipulator name not set, using default: ur5.");
        this->declare_parameter<std::string>("manipulator", "ur5");
    }
    this->declare_parameter("manipulator_name", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("manipulator_name")) {
        RCLCPP_WARN(this->get_logger(), "Manipulator name not set, using default: manipulator.");
        this->declare_parameter<std::string>("manipulator_name", "manipulator");
    }
    manipulator_      = this->get_parameter("manipulator").as_string();
    manipulator_name_ = this->get_parameter("manipulator_name").as_string();

    // Init command topic
    this->declare_parameter("command_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("command_topic")) {
        RCLCPP_WARN(this->get_logger(), "Command topic param not set, using default: /ur_rtde/controllers/joint_velocity_controller/command.");
        this->declare_parameter<std::string>("command_topic", "/ur_rtde/controllers/joint_velocity_controller/command");
    }
    command_topic_ = this->get_parameter("command_topic").as_string();

    // Init force feedback topic
    this->declare_parameter("force_feed_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("force_feed_topic")) {
        RCLCPP_WARN(this->get_logger(), "Force feedback topic param not set, using default: /ur_rtde/ft_sensor.");
        this->declare_parameter<std::string>("force_feed_topic", "/ur_rtde/ft_sensor");
    }
    force_feed_topic_ = this->get_parameter("force_feed_topic").as_string();

    // Zero force feed zero server
    this->declare_parameter("zero_ft_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("zero_ft_topic")) {
        RCLCPP_WARN(this->get_logger(), "Zero force feedback server name param not set, using default: /ur_rtde/zeroFTSensor.");
        this->declare_parameter<std::string>("zero_ft_topic", "/ur_rtde/zeroFTSensor");
    }
    zero_ft_sensor_topic_ = this->get_parameter("zero_ft_topic").as_string();
    // Init pose feedback topic
    this->declare_parameter("ee_pose_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("ee_pose_topic")) {
        RCLCPP_WARN(this->get_logger(), "EE pose topic param not set, using default: /ur_rtde/cartesian_pose.");
        this->declare_parameter<std::string>("ee_pose_topic", "/ur_rtde/cartesian_pose");
    }
    ee_pose_topic_ = this->get_parameter("ee_pose_topic").as_string();
    // Init vel feedback topic
    this->declare_parameter("ee_vel_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("ee_vel_topic")) {
        RCLCPP_WARN(this->get_logger(), "EE velocity feedback topic param not set, using default: /manipulator/tcp_vel.");
        this->declare_parameter<std::string>("ee_vel_topic", "/manipulator/tcp_vel");
    }
    ee_vel_topic_ = this->get_parameter("ee_vel_topic").as_string();

    // Initialize low-pass filter for the wrench
    this->declare_parameter("force_cut_freq", rclcpp::PARAMETER_DOUBLE);
    if (!this->has_parameter("force_cut_freq")) {
        RCLCPP_WARN(this->get_logger(), "Filter cut-out frequency not set, using default: 100 Hz.");
        this->declare_parameter<double>("force_cut_freq", 100.0);
    }
    this->force_cut_freq_ = this->get_parameter("force_cut_freq").as_double();

    // Init pushing interaction params
    this->declare_parameter("kp_push", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("push_force_goal", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("safe_push_dist", rclcpp::PARAMETER_DOUBLE);
    if (!this->has_parameter("kp_push")) {
        RCLCPP_WARN(this->get_logger(), "Proportional gain for pushing task not set, using default: 0.01.");
        this->declare_parameter<double>("kp_push", 0.1);
    }
    if (!this->has_parameter("push_force_goal")) {
        RCLCPP_WARN(this->get_logger(), "Goal force for pushing task not set, using default: 3.0.");
        this->declare_parameter<double>("push_force_goal", 3.0);
    }
    if (!this->has_parameter("safe_push_dist")) {
        RCLCPP_WARN(this->get_logger(), "Safety distance for pushing task not set, using default: 0.25.");
        this->declare_parameter<double>("safe_push_dist", 0.25);
    }
    double kp_push         = this->get_parameter("kp_push").as_double();
    double push_force_goal = this->get_parameter("push_force_goal").as_double();
    double safe_push_dist  = this->get_parameter("safe_push_dist").as_double();

    // Fill matrices values
    Eigen::Matrix<double, 6, 6> M_des = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> K_des = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> B_des = Eigen::Matrix<double, 6, 6>::Zero();

    for (uint i = 0; i < 6; i++) {
        M_des(i, i) = m_d[i];
        K_des(i, i) = k_d[i];
        B_des(i, i) = b_d[i];
    }

    // Create objects
    robot_kdl_ = std::make_shared<ManipulatorKDL>(manipulator_);
    adm_controller_ = std::make_shared<AdmittanceController>( M_des, K_des, B_des,
                                                              n_joints_, 1/loop_rate_,
                                                              dz_force_, dz_torque_,
                                                              kp_pos_, kp_rot_,
                                                              kp_push, push_force_goal,
                                                              safe_push_dist);
}

// ------------------------------------ UTILS -------------------------- ------- //


// ----------------------------- VARIABLE ADMITTANCE --------------------------- //
void AdmittanceControl::changeMassAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(0,0,new_params->x);
    adm_controller_->setAdmittanceParam(0,1,new_params->y);
    adm_controller_->setAdmittanceParam(0,2,new_params->z);
}

void AdmittanceControl::changeDampAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(1,0,new_params->x);
    adm_controller_->setAdmittanceParam(1,1,new_params->y);
    adm_controller_->setAdmittanceParam(1,2,new_params->z);
}

void AdmittanceControl::changeStiffAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(2,0,new_params->x);
    adm_controller_->setAdmittanceParam(2,1,new_params->y);
    adm_controller_->setAdmittanceParam(2,2,new_params->z);
}

void AdmittanceControl::changeMassRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(3,0,new_params->x);
    adm_controller_->setAdmittanceParam(3,1,new_params->y);
    adm_controller_->setAdmittanceParam(3,2,new_params->z);
}

void AdmittanceControl::changeDampRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(4,0,new_params->x);
    adm_controller_->setAdmittanceParam(4,1,new_params->y);
    adm_controller_->setAdmittanceParam(4,2,new_params->z);
}

void AdmittanceControl::changeStiffRotAdmittanceCallback(const std::shared_ptr<geometry_msgs::msg::Vector3> new_params)
{
    adm_controller_->setAdmittanceParam(5,0,new_params->x);
    adm_controller_->setAdmittanceParam(5,1,new_params->y);
    adm_controller_->setAdmittanceParam(5,2,new_params->z);
}

// --------------------- QUATERNIONS HANDLER -------------------
// Conversion from radians euler angles to quaternion
Eigen::Quaterniond AdmittanceControl::quaternion_from_euler(const double& roll, const double& pitch, const double& yaw)
{
    // Create the rotation matrix from Euler angles
    Eigen::AngleAxisd  rollAngle(roll,  Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd   yawAngle(yaw,   Eigen::Vector3d::UnitZ());

    // Combine the rotations into a single quaternion
    Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;

    // Normalize the quaternion (Eigen quaternions are not automatically normalized)
    quaternion.normalize();

    return quaternion;
}

// Conversion from quaternion to radians euler angles
Eigen::Vector3d AdmittanceControl::euler_from_quaternion(const Eigen::Quaterniond& quaternion)
{
    // Convert the quaternion to a rotation matrix
    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    // Extract the Euler angles (roll, pitch, yaw) from the rotation matrix
    Eigen::Vector3d euler_angles_rad = rotationMatrix.eulerAngles(2, 1, 0);  // ZYX order (yaw, pitch, roll)

    // Convert the angles from radians to degrees
    Eigen::Vector3d euler_angles;
    euler_angles[0] = euler_angles_rad[2];  // Roll
    euler_angles[1] = euler_angles_rad[1];  // Pitch
    euler_angles[2] = euler_angles_rad[0];  // Yaw

    // Check if angles are in the interval (-180,180]
    for (unsigned int k = 0; k < 3; k++)
    {
        if      (euler_angles[k] < -M_PI) { euler_angles[k] += 2*M_PI; }
        else if (euler_angles[k] > +M_PI) { euler_angles[k] -= 2*M_PI; }
    }

    return euler_angles;
}

// --------------------------- ROBOT STATE CALLBACKS -------------------------------

// Robot tcp pose callback
void AdmittanceControl::tcpPoseCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
{
    ee_pose_(0) = msg->position.x;
    ee_pose_(1) = msg->position.y;
    ee_pose_(2) = msg->position.z;
    // Eigen::Vector3d ee_pose_rpy = euler_from_quaternion(msg->pose.orientation);
    // ee_pose_(3) = ee_pose_rpy(0);
    // ee_pose_(4) = ee_pose_rpy(1);
    // ee_pose_(5) = ee_pose_rpy(2);
    ee_pose_(3) = msg->orientation.x;
    ee_pose_(4) = msg->orientation.y;
    ee_pose_(5) = msg->orientation.z;
    ee_pose_(6) = msg->orientation.w;

    // If manipulator_kdl has been chosen as planning interface
    // if (mode_bool_ == false) {robot_kdl_->fk(q_, ee_pose_);}
}

// Joint state callback
void AdmittanceControl::jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
{
    q_(0) = msg->position[0];
    q_(1) = msg->position[1];
    q_(2) = msg->position[2];
    q_(3) = msg->position[3];
    q_(4) = msg->position[4];
    q_(5) = msg->position[5];
}

// Robot twist callback
void AdmittanceControl::tcpTwistCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
    dx_(0) = msg->linear.x;
    dx_(1) = msg->linear.y;
    dx_(2) = msg->linear.z;
    dx_(3) = msg->angular.x;
    dx_(4) = msg->angular.y;
    dx_(5) = msg->angular.z;
}

// --------------------------- JACOBIAN COMPUTATIONS -----------------------------------
// Get Jacobian -> only in mode "kdl"
Eigen::MatrixXd AdmittanceControl::getJacobian()
{
    Eigen::MatrixXd jacobian_eigen(6, n_joints_);
    std::vector<std::vector<double>> jacobian(6, std::vector<double>(n_joints_));
    std::vector<double> q = {q_(0),q_(1),q_(2),q_(3),q_(4),q_(5)};

    robot_kdl_->jac(q, jacobian);

    for (uint i = 0; i < 6; i++)
    {
        for (int j = 0; j < n_joints_; j++)
        {
            jacobian_eigen(i, j) = jacobian[i][j];
        }
    }

    return jacobian_eigen;
}

// Get Inverse Jacobian -> only in mode "kdl"
Eigen::MatrixXd AdmittanceControl::getInvJacobian()
{
    return getJacobian().completeOrthogonalDecomposition().pseudoInverse();
}

// ---------------------------------- SENSORS -------------------------------- 

// Callback function for force sensor data
void AdmittanceControl::forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w)
{
    // Update wrench values with force and torque data
    wrench_(0) = w->force.x;
    wrench_(1) = w->force.y;
    wrench_(2) = w->force.z;
    wrench_(3) = w->torque.x;
    wrench_(4) = w->torque.y;
    wrench_(5) = w->torque.z;
}

Eigen::VectorXd AdmittanceControl::wrenchFilter(const Eigen::VectorXd& wrench)
{
    Eigen::VectorXd filtered_wrench = force_filter_->filter(wrench);

    geometry_msgs::msg::Wrench wrench_msg;
    wrench_msg.force.x  = filtered_wrench(0);
    wrench_msg.force.y  = filtered_wrench(1);
    wrench_msg.force.z  = filtered_wrench(2);
    wrench_msg.torque.x = filtered_wrench(3);
    wrench_msg.torque.y = filtered_wrench(4);
    wrench_msg.torque.z = filtered_wrench(5);

    wf_pub_->publish(wrench_msg);

    return filtered_wrench;
}

// ----------------------------- SETPOINT UPDATE -----------------------------
void AdmittanceControl::admittanceXdCallback(const std::shared_ptr<geometry_msgs::msg::Pose> p)
{
    xd_(0) = p->position.x;
    xd_(1) = p->position.y;
    xd_(2) = p->position.z;
    // Eigen::Vector3d x_rpy = euler_from_quaternion(msg->pose.orientation);
    // xd_(3) = x_rpy(0);
    // xd_(4) = x_rpy(1);
    // xd_(5) = x_rpy(2);
    xd_(3) = p->orientation.x;
    xd_(4) = p->orientation.y;
    xd_(5) = p->orientation.z;
    xd_(6) = p->orientation.w; 
}

// ----------------------------- ADMITTANCE ENABLER -----------------------------

// Service callback to enable or disable admittance control
bool AdmittanceControl::enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (req->data)
    {
        // Enable admittance control
        adm_controller_->enableAdmittance();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;

        // Trigger the service to zero the force-torque sensor
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        ft_client_->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                try
                {
                    auto response = future.get();
                    if (!response->success)
                    {
                        RCLCPP_WARN(this->get_logger(), "Force-torque sensor zeroing service failed.");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Force-torque sensor zeroing service succeeded.");
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            });
    }
    else
    {
        // Disable admittance control
        adm_controller_->disableAdmittance();
    }

    res->success = true;
    return true;
}

// Service callback to enable or disable admittance control
bool AdmittanceControl::enablePushRegulation(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                                   std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (req->data)
    {
        // Enable push regulation
        adm_controller_->enablePush();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }
    else
    {
        // Disable push regulation
        adm_controller_->disablePush();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }

    // Log the change for consistency
    RCLCPP_INFO(this->get_logger(), "Push regulation %s", req->data ? "enabled" : "disabled");

    res->success = true;
    return true;
}

// ----------------------------- MAIN FUNCTIONS -----------------------------

// Shutdown handler
void AdmittanceControl::shutdown_handler(int sig)
{
    // Show the result of the admittance control mean duration
    RCLCPP_INFO(rclcpp::get_logger("AdmittanceControl"), "Mean duration of the admittance control computations: %f", mean);
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for 1 second

    // Shutdown ROS2
    rclcpp::shutdown();
}

// Main ROS spinner
void AdmittanceControl::spinner()
{
    // Number of samples for mean computation
    unsigned long long int k = 0;           
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, shutdown_handler);

    // Set the loop rate
    rclcpp::Rate rate(loop_rate_);

    // ROS loop
	while (rclcpp::ok())
	{
        // Start loop time measurement
        auto start = this->now();

        // Listen to callbacks
        rclcpp::spin_some(this->get_node_base_interface());

        // Execute the main computations of the admittance control
        admittance_control_main();

        // Update mean computation
        // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());
        auto sample_k = (this->now() - start).seconds();
        k++;
        mean = 1.0 / static_cast<double>(k) * (sample_k + mean * static_cast<double>(k - 1));

        // Wait for the next iteration
		rate.sleep();
	}
}

// Main control loop function
void AdmittanceControl::admittance_control_main()
{
    if (mode_bool_ == false)    // If "kdl" mode is enables
    {
        // Compute the speed of the robot according to the given wrench
        Eigen::VectorXd filtered_wrench = wrenchFilter(wrench_);
        Eigen::VectorXd dq = adm_controller_->computeQSpeed(filtered_wrench,xd_,ee_pose_,dx_,dx_des_,ddx_des_,getJacobian());

        // Update tcp vel
        dx_ = getJacobian()*dq;

        // Convert the vel msg as ROS msg
        auto joint_vel = std_msgs::msg::Float64MultiArray();
        joint_vel.data.reserve(n_joints_);
        for (int i = 0; i < n_joints_; i++)    {joint_vel.data.push_back(dq(i));}

        // Send the command to the robot
        auto vel_pub = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>(vel_pub_);
        if (vel_pub) {
            vel_pub->publish(joint_vel);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast vel_pub_ to Float64MultiArray publisher.");
        }
    }
    else    // If "moveit" mode is enabled
    {
        // Compute the speed of as impedance control output
        Eigen::VectorXd filtered_wrench = wrenchFilter(wrench_);
        Eigen::VectorXd dx = adm_controller_->computeEESpeed(filtered_wrench,xd_,ee_pose_,dx_,dx_des_,ddx_des_);

        // Convert the vel msg as ROS msg
        auto ee_vel = geometry_msgs::msg::Twist();
        ee_vel.linear.x  = dx(0);
        ee_vel.linear.y  = dx(1);
        ee_vel.linear.z  = dx(2);
        ee_vel.angular.x = dx(3);
        ee_vel.angular.y = dx(4);
        ee_vel.angular.z = dx(5);

        // Send the command to the robot
        auto vel_pub = std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(vel_pub_);
        if (vel_pub) {
            vel_pub->publish(ee_vel);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to cast vel_pub_ to Twist publisher.");
        }
    }
}