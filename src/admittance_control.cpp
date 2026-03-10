#include "admittance_control/admittance_control.h"

// Constructor for the AdmittanceControl class
AdmittanceControl::AdmittanceControl(const std::string& node_name) : rclcpp::Node(node_name)
{

    // Update node params
    check_params();
    RCLCPP_INFO(this->get_logger(), "Params and attributes for admittance control are correctly initialized.");

    // --------- SUBSCRIBERS ------------
    // Subscribe to joint states and force sensor topics
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joints_state_topic_, 1, std::bind(&AdmittanceControl::jointCallback, this, std::placeholders::_1));
    force_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        force_feed_topic_, 1, std::bind(&AdmittanceControl::forceSensorCallback, this, std::placeholders::_1));
   
    // --------- PUBLISHERS -------------
    // Publish joint velocity command topic
    joint_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 1);
    
    // --------- SERVICES ----------------
    // Publish service to enable admittance control
    adm_service_ = this->create_service<std_srvs::srv::SetBool>(
        enable_adm_service_, std::bind(&AdmittanceControl::enableAdmittance, this, std::placeholders::_1, std::placeholders::_2));

    // Create service client to zero the force-torque sensor
    ft_client_ = this->create_client<std_srvs::srv::Trigger>(zero_ft_sensor_topic_);

    // Initialize wrench to zero
    wrench_.setZero();
    start_time_ = this->get_clock()->now();
}

AdmittanceControl::~AdmittanceControl(){
}

// Node params update
void AdmittanceControl::check_params()
{
    // Init joint names and number of joints
    this->declare_parameter("joint_names", rclcpp::PARAMETER_STRING_ARRAY);
    if (!this->has_parameter("joint_names")) {
        RCLCPP_WARN(this->get_logger(), "Joint names not set, using default names.");
        this->declare_parameter<std::vector<std::string>>("joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});
    }
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
    int n_joints = joint_names.size();

    // Init admittance control matrices
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
    double dz_force  = this->get_parameter("dz_force").as_double();
    double dz_torque = this->get_parameter("dz_torque").as_double();
    double kp_pos    = this->get_parameter("kp_pos").as_double();
    double kp_rot    = this->get_parameter("kp_rot").as_double();

    adm_controller_->setDeadZone(dz_force, dz_torque);
    Eigen::Matrix<double, 6, 6> internal_Kp = Eigen::Matrix<double, 6, 6>::Zero();
    internal_Kp(0,0) = kp_pos;
    internal_Kp(1,1) = kp_pos;
    internal_Kp(2,2) = kp_pos;
    internal_Kp(3,3) = kp_rot;
    internal_Kp(4,4) = kp_rot;
    internal_Kp(5,5) = kp_rot;

    adm_controller_->changeInternalP(internal_Kp);
    

    // Init control params
    this->declare_parameter("loop_rate", rclcpp::PARAMETER_DOUBLE);
    if (!this->has_parameter("loop_rate")) {
        RCLCPP_WARN(this->get_logger(), "Loop rate not set, using default: 500 Hz.");
        this->declare_parameter<double>("loop_rate", 500.0);
    }
    double loop_rate = this->get_parameter("loop_rate").as_double();

    this->declare_parameter("manipulator_name", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("manipulator_name")) {
        RCLCPP_WARN(this->get_logger(), "Manipulator name not set, using default: manipulator.");
        this->declare_parameter<std::string>("manipulator_name", "manipulator");
    }
    std::string manipulator_name = this->get_parameter("manipulator_name").as_string();
    
    // ----------------- TOPICS ---------------------
    // Joint States topic
    this->declare_parameter("joint_state_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("joint_state_topic")) {
        RCLCPP_WARN(this->get_logger(), "Joint States topic param not set, using default: /joint_states");
        this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    }
    joints_state_topic_ = this->get_parameter("joint_state_topic").as_string();

    // Init force feedback topic
    this->declare_parameter("force_feed_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("force_feed_topic")) {
        RCLCPP_WARN(this->get_logger(), "Force feedback topic param not set, using default: /ur_rtde/ft_sensor.");
        this->declare_parameter<std::string>("force_feed_topic", "/ur_rtde/ft_sensor");
    }
    force_feed_topic_ = this->get_parameter("force_feed_topic").as_string();

    // Init command topic
    this->declare_parameter("command_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("command_topic")) {
        RCLCPP_WARN(this->get_logger(), "Command topic param not set, using default: /ur_rtde/controllers/joint_velocity_controller/command.");
        this->declare_parameter<std::string>("command_topic", "/ur_rtde/controllers/joint_velocity_controller/command");
    }
    command_topic_ = this->get_parameter("command_topic").as_string();

    // Init enable admittance service call
    this->declare_parameter("enable_adm_service", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("enable_adm_service")) {
        RCLCPP_WARN(this->get_logger(), "Enable admittance service call param not set, using default: /enable_admittance.");
        this->declare_parameter<std::string>("enable_adm_service", "/enable_admittance");
    }
    enable_adm_service_ = this->get_parameter("enable_adm_service").as_string();

    // Zero force feed zero server
    this->declare_parameter("zero_ft_topic", rclcpp::PARAMETER_STRING);
    if (!this->has_parameter("zero_ft_topic")) {
        RCLCPP_WARN(this->get_logger(), "Zero force feedback server name param not set, using default: /ur_rtde/zeroFTSensor.");
        this->declare_parameter<std::string>("zero_ft_topic", "/ur_rtde/zeroFTSensor");
    }
    zero_ft_sensor_topic_ = this->get_parameter("zero_ft_topic").as_string();


    // // Initialize low-pass filter for the wrench
    // this->declare_parameter("force_cut_freq", rclcpp::PARAMETER_DOUBLE);
    // if (!this->has_parameter("force_cut_freq")) {
    //     RCLCPP_WARN(this->get_logger(), "Filter cut-out frequency not set, using default: 100 Hz.");
    //     this->declare_parameter<double>("force_cut_freq", 100.0);
    // }
    // this->force_cut_freq_ = this->get_parameter("force_cut_freq").as_double();

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
    adm_controller_ = std::make_shared<AdmittanceController>( M_des, K_des, B_des, manipulator_name , n_joints , 1/loop_rate );
}

// Callback function for joint states
void AdmittanceControl::jointCallback(const std::shared_ptr<sensor_msgs::msg::JointState> js)
{
    joint_pos_ = js->position;
    // Update joint positions in the admittance controller
    adm_controller_->updateJoints(js->position);
    // std::cout << "-----------2" << std::endl;

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
    // std::cout << "-----------3" << std::endl;

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

    std_msgs::msg::Float64MultiArray joint_vel;
    for (uint i = 0; i < dq.rows(); i++)
        joint_vel.data.push_back(dq(i, 0));

    // Publish velocities
    joint_vel_pub_->publish(joint_vel);
}