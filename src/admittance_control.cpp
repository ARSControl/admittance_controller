#include "admittance_control/admittance_control.h"

// Constructor for the AdmittanceControl class
AdmittanceControl::AdmittanceControl(): Node("admittance_control_node")
{
    // Update node params
    check_params();

    // --------- SUBSCRIBERS ------------
    rclcpp::SubscriptionOptions sub_options;

    // TCP pose subscriber
    auto cb_group_tcp_pose = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_tcp_pose;
    this->declare_parameter("tcp_pose_topic", manipulator_name_+"tcp_pose");
    std::string tcp_pose_topic = this->get_parameter("tcp_pose_topic").as_string();
    tcp_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(tcp_pose_topic, 1,
                 std::bind(&AdmittanceControl::tcpPoseCallback, this, std::placeholders::_1), sub_options);

    // TCP velocity subscriber
    auto cb_group_tcp_vel = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_tcp_vel;
    this->declare_parameter("tcp_vel_topic", manipulator_name_+"tcp_vel");
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
    xd_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(manipulator_name_+"/adm_xd", 1,
                 std::bind(&AdmittanceControl::admittanceXdCallback, this, std::placeholders::_1), sub_options);

    // Subscribers to change admittance
    auto cb_group_adm = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = cb_group_adm;
    m_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeMassAdmittanceCallback, this, std::placeholders::_1),sub_options);
    b_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeDampAdmittanceCallback, this, std::placeholders::_1),sub_options);
    k_adm_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_pos", 1,
                        std::bind(&AdmittanceControl::changeStiffAdmittanceCallback, this, std::placeholders::_1),sub_options);
    m_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeMassRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    b_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeDampRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    k_adm_rot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_rot", 1,
                        std::bind(&AdmittanceControl::changeStiffRotAdmittanceCallback, this, std::placeholders::_1),sub_options);
    
    // --------- PUBLISHERS -------------
    
    // Publish EE velocity topic
    this->declare_parameter("command_topic", manipulator_name_+"/cmd_vel");
    std::string cart_vel_topic = this->get_parameter("command_topic").as_string();
    cartesian_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cart_vel_topic, 1);

    // Publish filtered force-torque sensor topic
    wf_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>(manipulator_name_+"/filtered_wrench", 1);
    
    // --------- SERVICES ----------------
    // Publish service to enable admittance control
    adm_service_ = this->create_service<std_srvs::srv::SetBool>(manipulator_name_+"/enable_admittance",
                   std::bind(&AdmittanceControl::enableAdmittance, this, std::placeholders::_1, std::placeholders::_2));

    // Publish service to enable pushing control
    push_service_ = this->create_service<std_srvs::srv::SetBool>(manipulator_name_+"/enable_push_regulation",
                   std::bind(&AdmittanceControl::enablePush, this, std::placeholders::_1, std::placeholders::_2));

    // Create service client to zero the force-torque sensor
    this->declare_parameter("zero_ft_topic", "/ur_rtde/zeroFTSensor");
    std::string zero_ft_sensor_topic = this->get_parameter("zero_ft_topic").as_string();
    ft_client_ = this->create_client<std_srvs::srv::Trigger>(zero_ft_sensor_topic);

    // ---------------- SHUTDOWN HANDLER ----------------
    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&AdmittanceControl::shutdown_handler, this) // Register shutdown handler
    );

    // --------------- INITIALIZATION----------------
    // Initialize wrench to zero
    wrench_  = Eigen::VectorXd::Zero(6);
    ee_pose_ = Eigen::VectorXd::Zero(7);
    xd_      = Eigen::VectorXd::Zero(7);
    dx_      = Eigen::VectorXd::Zero(6);
    dx_des_  = Eigen::VectorXd::Zero(6);
    ddx_des_ = Eigen::VectorXd::Zero(6);
}

// Node params update
void AdmittanceControl::check_params()
{
    // Declare and get joint names
    this->declare_parameter("n_joints", 6);
    n_joints_ = this->get_parameter("n_joints").as_int();
    this->declare_parameter("manipulator_name", "manipulator");
    manipulator_name_ = this->get_parameter("manipulator_name").as_string();

    // Declare and get diagonal mass, damping, stiffness
    this->declare_parameter("m_d", std::vector<double>{ 5.0,  5.0,  5.0, 0.1, 0.1, 0.1});
    this->declare_parameter("k_d", std::vector<double>{50.0, 50.0, 50.0, 1.0, 1.0, 1.0});
    this->declare_parameter("b_d", std::vector<double>{25.0, 25.0, 25.0, 0.5, 0.5, 0.5});
    m_d = this->get_parameter("m_d").as_double_array();
    k_d = this->get_parameter("k_d").as_double_array();
    b_d = this->get_parameter("b_d").as_double_array();

    this->declare_parameter("m_d_block", std::vector<double>{ 5.0,  5.0,  5.0, 0.1, 0.1, 0.1});
    this->declare_parameter("k_d_block", std::vector<double>{50.0, 50.0, 50.0, 1.0, 1.0, 1.0});
    this->declare_parameter("b_d_block", std::vector<double>{25.0, 25.0, 25.0, 0.5, 0.5, 0.5});
    m_d_block = this->get_parameter("m_d_block").as_double_array();
    k_d_block = this->get_parameter("k_d_block").as_double_array();
    b_d_block = this->get_parameter("b_d_block").as_double_array();

    // Declare and get controller gains
    this->declare_parameter("kp_pos", 0.0);
    this->declare_parameter("kp_rot", 0.0);
    this->declare_parameter("kp_push", 0.001);
    this->declare_parameter("push_force_goal", 8.0);
    this->declare_parameter("safe_push_dist", 0.2);

    double kp_pos = this->get_parameter("kp_pos").as_double();
    double kp_rot = this->get_parameter("kp_rot").as_double();
    double kp_push = this->get_parameter("kp_push").as_double();
    push_force_goal_ = this->get_parameter("push_force_goal").as_double();
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

    // Create AdmittanceController object
    adm_controller_ = new AdmittanceController(
        m_d, k_d, b_d,
        n_joints_, 1.0 / loop_rate_,
        dz_force, dz_torque,
        kp_pos, kp_rot,
        kp_push, push_force_goal_,
        safe_push_dist, acc_filter_freq
    );

    // Init force filter
    force_filter_ = new filters::RCFilter(6, acc_filter_freq, 1.0 / loop_rate_);
}

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

// ----------------------------- VARIABLE PUSHING ADMITTANCE CONTROL --------------------------- //
void AdmittanceControl::updateParamOnWrenchError(const Eigen::VectorXd& wrench, const double& reference_force)
{
    // Adjust this value to control the sensitivity of the update
    double lambda = 1.0;

    // Iterate over the force components to change the desired admittance mass 
    for (unsigned int k = 0; k < 1; k++)
    {
        // Calculate the difference between the current force and the reference force
        double delta_force = std::abs(std::abs(wrench(k)) - std::abs(reference_force));
        // Update the mass based on the difference
        double mass = (m_d_block[k] - m_d[k]) * std::exp(-lambda * delta_force) + m_d[0];
        adm_controller_->setAdmittanceParam(0,k,mass);
        // Update the damping based on the difference
        double damping = (b_d_block[k] - b_d[k]) * std::exp(-lambda * delta_force) + b_d[0];
        adm_controller_->setAdmittanceParam(1,k,damping);
        // Update the stiffness based on the difference
        double stiffness = (k_d[k] - k_d_block[k]) *(1 - std::exp(-lambda * delta_force)) + k_d_block[0];
        adm_controller_->setAdmittanceParam(2,k,stiffness);
    } 
}

// --------------------- QUATERNIONS HANDLER ------------------- //
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

// --------------------------- ROBOT STATE CALLBACKS ------------------------------- //

// Robot tcp pose callback
void AdmittanceControl::tcpPoseCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
{
    // Update end-effector pose
    ee_pose_(0) = msg->pose.position.x;
    ee_pose_(1) = msg->pose.position.y;
    ee_pose_(2) = msg->pose.position.z;
    // Eigen::Vector3d ee_pose_rpy = euler_from_quaternion(msg->pose.orientation);
    // ee_pose_(3) = ee_pose_rpy(0);
    // ee_pose_(4) = ee_pose_rpy(1);
    // ee_pose_(5) = ee_pose_rpy(2);
    ee_pose_(3) = msg->pose.orientation.x;
    ee_pose_(4) = msg->pose.orientation.y;
    ee_pose_(5) = msg->pose.orientation.z;
    ee_pose_(6) = msg->pose.orientation.w;
}

// Robot twist callback
void AdmittanceControl::tcpVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
    dx_(0) = msg->linear.x;
    dx_(1) = msg->linear.y;
    dx_(2) = msg->linear.z;
    dx_(3) = msg->angular.x;
    dx_(4) = msg->angular.y;
    dx_(5) = msg->angular.z;
}

// ---------------------------------- SENSORS -------------------------------- //

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

// ----------------------------- SETPOINT UPDATE ----------------------------- //
void AdmittanceControl::admittanceXdCallback(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
{
    xd_(0) = msg->position.x;
    xd_(1) = msg->position.y;
    xd_(2) = msg->position.z;
    // Eigen::Vector3d x_rpy = euler_from_quaternion(msg->pose.orientation);
    // xd_(3) = x_rpy(0);
    // xd_(4) = x_rpy(1);
    // xd_(5) = x_rpy(2);
    xd_(3) = msg->orientation.x;
    xd_(4) = msg->orientation.y;
    xd_(5) = msg->orientation.z;
    xd_(6) = msg->orientation.w; 
}

// ----------------------------- SERVICE CALLBACKS ----------------------------- //
// Service callback to enable or disable admittance control
void AdmittanceControl::enableAdmittance(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                         std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    if (request->data)
    {
        // Enable admittance control
        adm_controller_->enableAdmittance();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }
    else
    {
        // Disable admittance control
        adm_controller_->disableAdmittance();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }
    RCLCPP_INFO(this->get_logger(), "Admittance control %s", request->data ? "ENABLED" : "DISABLED");

    response->success = true;
    response->message = request->data ? "Admittance control enabled" : "Admittance control disabled";
}

// Service callback to enable or disable push regulation
void AdmittanceControl::enablePush(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                   std::shared_ptr<std_srvs::srv::SetBool::Response>      response)
{
    if (request->data)
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
    RCLCPP_INFO(this->get_logger(), "Push regulation %s", request->data ? "ENABLED" : "DISABLED");

    response->success = true;
    response->message = request->data ? "Push regulation enabled" : "Push regulation disabled";
}

// ----------------------------- MAIN LOOP ---------------------------- //
// Main loop to compute and publish joint velocities
void AdmittanceControl::computeAdmittanceControl()
{   
    // Filter wrench measurements
    Eigen::VectorXd filtered_wrench = wrenchFilter(wrench_);

    // Adapt parameters if the force error is not zero
    updateParamOnWrenchError(filtered_wrench, push_force_goal_);

    // Compute the desired end-effector speed according to the admittance controller
    Eigen::VectorXd dx_cmd = adm_controller_->computeEESpeed(filtered_wrench,xd_,ee_pose_,dx_,dx_des_,ddx_des_);

    // Publish the ee speed 
    geometry_msgs::msg::Twist cartesian_vel;
    cartesian_vel.linear.x  = dx_cmd(0);
    cartesian_vel.linear.y  = dx_cmd(1);
    cartesian_vel.linear.z  = dx_cmd(2);
    cartesian_vel.angular.x = dx_cmd(3);
    cartesian_vel.angular.y = dx_cmd(4);
    cartesian_vel.angular.z = dx_cmd(5);
    cartesian_vel_pub_->publish(cartesian_vel);
}

// ------------------------------ SPINNER ----------------------------- //
void AdmittanceControl::shutdown_handler()
{
    RCLCPP_INFO(get_logger(), "Admittance controller mean time: %f s", spinner_mean_);
}

void AdmittanceControl::spinner()
{
    // Add the node to the executor
    executor_.add_node(shared_from_this());

    // Create a steady clock to measure time
	rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // Create timer callback with specified frequency (loop_rate_ in Hz)
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_rate_),  // period in seconds
        [this, &steady_clock]() {

                // This is the main loop for the node
                auto start_time = steady_clock.now();
                // Compute admittance control
                computeAdmittanceControl();
                // Calculate the mean time for each iteration of the spinner
                double elapsed_time = (steady_clock.now() - start_time).seconds();
                spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples) + elapsed_time) / static_cast<double>(num_samples + 1);
                num_samples++;
            });

    // Start spinning
    executor_.spin();

    // Shutdown the executor
    rclcpp::shutdown();
}
