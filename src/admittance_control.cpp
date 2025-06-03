#include "admittance_control/admittance_control.h"

// Constructor for the AdmittanceControl class
AdmittanceControl::AdmittanceControl(const std::string& node_name) : rclcpp::Node(node_name)
{
    // Update node params
    check_params();

    // --------- SUBSCRIBERS ------------
    // Joint state subscriber
    auto cb_group_js = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options; sub_options.callback_group = cb_group_js;
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",1,
                 std::bind(&AdmittanceControl::jointCallback, this, std::placeholders::_1), sub_options);

    // TCP pose subscriber
    auto cb_group_tcp_pose = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_tcp_pose;
    this->declare_parameter("tcp_pose_topic", manipulator_name+"tcp_pose");
    std::string tcp_pose_topic = this->get_parameter("tcp_pose_topic").as_string();
    tcp_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(tcp_pose_topic, 1,
                 std::bind(&AdmittanceControl::tcpPoseCallback, this, std::placeholders::_1), sub_options);

    // TCP velocity subscriber
    auto cb_group_tcp_vel = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_tcp_vel;
    this->declare_parameter("tcp_vel_topic", manipulator_name+"tcp_vel");
    std::string tcp_vel_topic = this->get_parameter("tcp_vel_topic").as_string();
    tcp_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(tcp_vel_topic, 1,
                 std::bind(&AdmittanceControl::tcpVelCallback, this, std::placeholders::_1), sub_options);

    // Declare and get force sensor topic
    auto cb_group_force = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_force;
    this->declare_parameter("force_feed_topic", "/ur_rtde/ft_sensor");
    std::string force_feed_topic = this->get_parameter("force_feed_topic").as_string();
    force_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(force_feed_topic,1,
                 std::bind(&AdmittanceControl::forceSensorCallback, this, std::placeholders::_1), sub_options);

    // Declare and get reference pose topic
    auto cb_group_ref_pose = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_ref_pose;
    xd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(manipulator_name+"/adm_xd", 1,
                 std::bind(&AdmittanceControl::admittanceXdCallback, this, std::placeholders::_1), sub_options);

    // Subscribers to change admittance
    auto cb_group_adm = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_adm;
    m_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/m_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeMassAdmittanceCallback, this, std::placeholders::_1),sub_options);
    b_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/b_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeDampAdmittanceCallback, this, std::placeholders::_1),sub_options);
    k_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/k_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeStiffAdmittanceCallback, this, std::placeholders::_1),sub_options);
    m_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/m_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeMassRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    b_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/b_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeDampRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    k_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name + "/k_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeStiffRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    
    // --------- PUBLISHERS -------------
    // Publish EE velocity topic
    this->declare_parameter("command_topic", manipulator_name+"/cmd_vel");
    std::string cart_vel_topic = this->get_parameter("command_topic").as_string();
    cartesian_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cart_vel_topic, 1);

    // Publish filtered force-torque sensor topic
    wf_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(manipulator_name+"/filtered_wrench", 1);
    
    // --------- SERVICES ----------------
    // Publish service to enable admittance control
    adm_service_ = this->create_service<std_srvs::srv::SetBool>(manipulator_name+"/enable_admittance",
                   std::bind(&AdmittanceControl::enableAdmittance, this, std::placeholders::_1, std::placeholders::_2));

    // Publish service to enable pushing control
    push_service_ = this->create_service<std_srvs::srv::SetBool>(manipulator_name+"/enable_push_regulation",
                   std::bind(&AdmittanceControl::enablePush, this, std::placeholders::_1, std::placeholders::_2));

    // Create service client to zero the force-torque sensor
    this->declare_parameter("zero_ft_topic", "/ur_rtde/zeroFTSensor");
    std::string zero_ft_sensor_topic = this->get_parameter("zero_ft_topic").as_string();
    ft_client_ = this->create_client<std_srvs::srv::Trigger>(zero_ft_sensor_topic);

    // --------------- INITIALIZATION----------------
    // Initialize wrench to zero
    wrench_  = Eigen::VectorXd::Zero(7);
    ee_pose_ = Eigen::VectorXd::Zero(7);
    xd_      = Eigen::VectorXd::Zero(7);
    dx_      = Eigen::VectorXd::Zero(6);
    dx_des_  = Eigen::VectorXd::Zero(6);
    ddx_des_ = Eigen::VectorXd::Zero(6);
    q_       = Eigen::VectorXd::Zero(n_joints_);
}

// Node params update
void AdmittanceControl::check_params()
{
    // Declare and get joint names
    this->declare_parameter("n_joints", 6);
    n_joints_ = this->get_parameter("n_joints").as_int();
    this->declare_parameter("manipulator_name", "manipulator");
    std::string manipulator_name = this->get_parameter("manipulator_name").as_string();

    // Declare and get diagonal mass, damping, stiffness
    this->declare_parameter("m_d", std::vector<double>{12.5, 12.5, 12.5, 0.25, 0.25, 0.25});
    this->declare_parameter("k_d", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter("b_d", std::vector<double>{25.0, 25.0, 25.0, 1.5, 1.5, 1.5});
    std::vector<double> m_d = this->get_parameter("m_d").as_double_array();
    std::vector<double> k_d = this->get_parameter("k_d").as_double_array();
    std::vector<double> b_d = this->get_parameter("b_d").as_double_array();

    // Declare and get controller gains
    this->declare_parameter("kp_pos", 0.0);
    this->declare_parameter("kp_rot", 0.0);
    this->declare_parameter("kp_push", 0.001);
    this->declare_parameter("push_force_goal", 8.0);
    this->declare_parameter("safe_push_dist", 0.2);

    double kp_pos = this->get_parameter("kp_pos").as_double();
    double kp_rot = this->get_parameter("kp_rot").as_double();
    double kp_push = this->get_parameter("kp_push").as_double();
    double push_force_goal = this->get_parameter("push_force_goal").as_double();
    double safe_push_dist = this->get_parameter("safe_push_dist").as_double();

    // Declare and get dead zone thresholds
    this->declare_parameter("dz_force", 5.0);
    this->declare_parameter("dz_torque", 0.3);
    double dz_force  = this->get_parameter("dz_force").as_double();
    double dz_torque = this->get_parameter("dz_torque").as_double();

    // Declare and get frequency
    this->declare_parameter("force_cut_freq", 100.0);
    this->declare_parameter("loop_rate", 500.0);
    double acc_filter_freq = this->get_parameter("force_cut_freq").as_double();
    loop_rate_             = this->get_parameter("loop_rate").as_double();

    // Fill matrices
    Eigen::Matrix M_des = Eigen::MatrixXd::Zero(n_joints);
    Eigen::Matrix K_des = Eigen::MatrixXd::Zero(n_joints);
    Eigen::Matrix B_des = Eigen::MatrixXd::Zero(n_joints);
    for (int i = 0; i < n_joints; i++) { M_des(i, i) = m_d[i]; K_des(i, i) = k_d[i]; B_des(i, i) = b_d[i]; }

    // Create AdmittanceController object
    adm_controller_ = std::make_shared<AdmittanceController>(
        M_des, K_des, B_des,
        n_joints, 1.0 / loop_rate,
        dz_force, dz_torque,
        kp_pos, kp_rot,
        kp_push, push_force_goal,
        safe_push_dist, acc_filter_freq
    );

    // Init force filter
    force_filter_ = new filters::RCFilter(6, acc_filter_freq, 1.0 / loop_rate_);

}

// ----------------------------- VARIABLE ADMITTANCE --------------------------- //
void AdmittanceControl::changeMassAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
{
    adm_controller_->setAdmittanceParam(0,0,new_params->x);
    adm_controller_->setAdmittanceParam(0,1,new_params->y);
    adm_controller_->setAdmittanceParam(0,2,new_params->z);
}

void AdmittanceControl::changeDampAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
{
    adm_controller_->setAdmittanceParam(1,0,new_params->x);
    adm_controller_->setAdmittanceParam(1,1,new_params->y);
    adm_controller_->setAdmittanceParam(1,2,new_params->z);
}

void AdmittanceControl::changeStiffAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
{
    adm_controller_->setAdmittanceParam(2,0,new_params->x);
    adm_controller_->setAdmittanceParam(2,1,new_params->y);
    adm_controller_->setAdmittanceParam(2,2,new_params->z);
}

void AdmittanceControl::changeMassRotAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
{
    adm_controller_->setAdmittanceParam(3,0,new_params->x);
    adm_controller_->setAdmittanceParam(3,1,new_params->y);
    adm_controller_->setAdmittanceParam(3,2,new_params->z);
}

void AdmittanceControl::changeDampRotAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
{
    adm_controller_->setAdmittanceParam(4,0,new_params->x);
    adm_controller_->setAdmittanceParam(4,1,new_params->y);
    adm_controller_->setAdmittanceParam(4,2,new_params->z);
}

void AdmittanceControl::changeStiffRotAdmittanceCallback(const geometry_msgs::Vector3::ConstPtr& new_params)
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



// CONTINUE FROM HERE



// Callback function for joint states
void AdmittanceControl::jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> js)
{
    joint_pos_ = js->position;
    // Update joint positions in the admittance controller
    adm_controller_->updateJoints(js->position);
}

// Callback function for force sensor data
void AdmittanceControl::forceSensorCallback(const std::shared_ptr<geometry_msgs::msg::Wrench> w)
{
    // Update wrench values with force and torque data
    wrench_(0, 0) = w->force.x;
    wrench_(1, 0) = w->force.y;
    wrench_(2, 0) = w->force.z;
    wrench_(3, 0) = w->torque.x;
    wrench_(4, 0) = w->torque.y;
    wrench_(5, 0) = w->torque.z;
}

// Service callback to enable or disable admittance control
bool AdmittanceControl::enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
    if (req->data)
    {
        // Enable admittance control and zero the force-torque sensor
        adm_controller_->enableAdmittance();
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

// Main loop to compute and publish joint velocities
void AdmittanceControl::spinner()
{   
    rclcpp::spin_some(this->get_node_base_interface());
    auto dq = adm_controller_->computeSpeed(wrench_);

    auto dx = adm_controller_->returnTwist();
    std_msgs::msg::Float64MultiArray joint_vel;
    geometry_msgs::msg::Twist cartesian_vel;
    for (uint i = 0; i < dq.rows(); i++)
        joint_vel.data.push_back(dq(i, 0));

    cartesian_vel.linear.x  = dx(0);
    cartesian_vel.linear.y  = dx(1);
    cartesian_vel.linear.z  = dx(2);
    cartesian_vel.angular.x = dx(3);
    cartesian_vel.angular.y = dx(4);
    cartesian_vel.angular.z = dx(5);

    if(data_logger_enabled_){
        auto now = this->get_clock()->now();

        data_.emplace_back( 
            (now.nanoseconds() - start_time_.nanoseconds()) / 1e6,
            dq(0,0),
            dq(1,0),
            dq(2,0),
            dq(3,0),
            dq(4,0),
            dq(5,0),
            joint_pos_[0],
            joint_pos_[1],
            joint_pos_[2],
            joint_pos_[3],
            joint_pos_[4],
            joint_pos_[5]
        );
    }

    // Publish velocities
    joint_vel_pub_->publish(joint_vel);
    cartesian_vel_pub_->publish(cartesian_vel);
}
