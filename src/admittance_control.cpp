#include "admittance_control/admittance_control.h"

// Constructor for the AdmittanceControl class
AdmittanceControl::AdmittanceControl(const std::string& node_name) : rclcpp::Node(node_name)
{
    // Subscribe to joint states and force sensor topics
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&AdmittanceControl::jointCallback, this, std::placeholders::_1));
    force_sub_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/ur_rtde/ft_sensor", 1, std::bind(&AdmittanceControl::forceSensorCallback, this, std::placeholders::_1));

    // Variable admittance control
    var_adm_sub_ = this->create_subscription<energy_tank::msg::InertiaDamping>(
        "/current_inertia_damping", 1, std::bind(&AdmittanceControl::inertiaDampingCallback, this, std::placeholders::_1));

    // Advertise joint velocity command topic
    joint_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ur_rtde/controllers/joint_velocity_controller/command", 1);

    // Advertise EE velocity topic
    cartesian_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cartesian_velocity", 1);

    // Advertise service to enable admittance control
    adm_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/enable_admittance", std::bind(&AdmittanceControl::enableAdmittance, this, std::placeholders::_1, std::placeholders::_2));

    // Create service client to zero the force-torque sensor
    ft_client_ = this->create_client<std_srvs::srv::Trigger>("/ur_rtde/zeroFTSensor");

    // Initialize wrench to zero
    wrench_.setZero();

    // Define desired mass, damping, and stiffness matrices
    Eigen::MatrixXd Mdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Bdes = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd Kdes = Eigen::MatrixXd::Zero(6, 6);
    std::string manipulator_name = "ur10e";
    int n_joints = 6;
    double ts = 0.002;
    // std::vector<double> mass = {2.5, 2.5, 2.5, 0.05, 0.05, 0.05};
    // std::vector<double> damping = {5.0, 5.0, 5.0, 0.3, 0.3, 0.3};
    std::vector<double> mass = {5.0,   5.0,  5.0,  0.1,  0.1,  0.1};
    std::vector<double> damping = {25.0, 25.0, 25.0,  0.5,  0.5,  0.5};

    // Initialize mass and damping matrices
    for (uint i = 0; i < 6; i++)
    {
        // Mdes(i, i) = 4.0 * mass[i];
        // Bdes(i, i) = 4.0 * damping[i];
        Mdes(i, i) = mass[i];
        Bdes(i, i) = damping[i];
    }

    // Create an instance of AdmittanceController
    adm_controller_ = std::make_shared<AdmittanceController>(Mdes, Kdes, Bdes, manipulator_name, n_joints, ts);

    // Initialize wrench to zero
    wrench_.setZero();

    start_time_ = this->get_clock()->now();

    // std::cout << "-----------1" << std::endl;
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

// Callback function for changin Inertia and Damping matrice
void AdmittanceControl::inertiaDampingCallback(const std::shared_ptr<energy_tank::msg::InertiaDamping> new_params)
{
    // Extract new params
    Vector6d new_inertia = Vector6d( new_params->inertia[0] , new_params->inertia[1] , new_params->inertia[2] ,
                                     new_params->inertia[3] , new_params->inertia[4] , new_params->inertia[5] );
    Vector6d new_damping = Vector6d( new_params->damping[0] , new_params->damping[1] , new_params->damping[2] ,
                                     new_params->damping[3] , new_params->damping[4] , new_params->damping[5] );

    Matrix6d new_Mdes = new_inertia.asDiagonal();
    Matrix6d new_Ddes = new_damping.asDiagonal();

    // std::cout << "-----------4" << std::endl;


    adm_controller_->changeParameters( new_Mdes , new_Ddes );
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
    // std::cout << "-----------5" << std::endl;

    rclcpp::spin_some(this->get_node_base_interface());
    auto dq = adm_controller_->computeSpeed(wrench_);

    // std::cout << "+++++++++++++++++++++++**QUI" << std::endl;
    // std::cout << "dq: " << dq << std::endl;

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

    // Publish velocities
    joint_vel_pub_->publish(joint_vel);
    cartesian_vel_pub_->publish(cartesian_vel);
}

void AdmittanceControl::writeToCSV(){

    // Specify the full path to the CSV file
    std::string file_path = "logged_data_adm.csv";
    std::ofstream file;
    file.open(file_path);

    // Write headers
    file << "timestamp,joint_vel_0,joint_vel_1,joint_vel_2,joint_vel_3,joint_vel_4,joint_vel_5,joint_pos_0,joint_pos_1,joint_pos_2,joint_pos_3,joint_pos_4,joint_pos_5\n";

    // Write the data
    for (const auto& entry : data_)
    {
        file << std::get<0>(entry) << ","; // Timestamp

        // Write each data field, using value_or to provide a default empty value if the optional is not set
        file << std::get<1>(entry).value_or(0.0) << ","; // 
        file << std::get<2>(entry).value_or(0.0) << ","; // 
        file << std::get<3>(entry).value_or(0.0) << ","; // 
        file << std::get<4>(entry).value_or(0.0) << ","; // 
        file << std::get<5>(entry).value_or(0.0) << ","; // 
        file << std::get<6>(entry).value_or(0.0) << ","; // 
        file << std::get<7>(entry).value_or(0.0) << ","; // 
        file << std::get<8>(entry).value_or(0.0) << ","; // 
        file << std::get<9>(entry).value_or(0.0) << ","; // 
        file << std::get<10>(entry).value_or(0.0) << ","; //
        file << std::get<11>(entry).value_or(0.0) << ","; //
        file << std::get<12>(entry).value_or(0.0) << "\n"; //
    }

    file.close();
}