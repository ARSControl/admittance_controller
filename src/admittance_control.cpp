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

#include "admittance_control/admittance_control.h"

// --------------------- STATIC VARIABLES ---------------------
double AdmittanceControl::mean = 0.0;  // variable for the control time average computation

// --------------------- PUBLIC CONSTRUCTOR ---------------------
AdmittanceControl::AdmittanceControl(const std::string& node_name)
{
    // Initialize wrench to zero
    wrench_      = Eigen::VectorXd::Zero(6);
    force_limit_ = Eigen::VectorXd::Zero(6);

    // Initialize state vectors
    ee_pose_ = Eigen::VectorXd::Zero(7);    // 7 for position + quaternion
    xd_      = Eigen::VectorXd::Zero(7);    // 7 for position + quaternion
    dx_      = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity
    dx_des_  = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity
    ddx_des_ = Eigen::VectorXd::Zero(6);    // 6 for linear and angular velocity

    // Update node params
    node_name_ = node_name;
    check_params();
    ROS_INFO("Params and attributes for admittance controll are correctly initialized.");

    // Subscribe to topics
    joint_sub_ = nh_.subscribe("/joint_states",             1, &AdmittanceControl::jointCallback,       this);
    force_sub_ = nh_.subscribe(force_feed_topic_,           1, &AdmittanceControl::forceSensorCallback, this);
    xd_sub_    = nh_.subscribe(manipulator_name_+"/adm_xd", 1, &AdmittanceControl::admittanceXdCallback, this);
    
    // Subscribers to change admittance
    m_adm_pos_sub_ = nh_.subscribe(manipulator_name_+"/m_adm_pos", 1, &AdmittanceControl::changeMassAdmittanceCallback, this);
    b_adm_pos_sub_ = nh_.subscribe(manipulator_name_+"/b_adm_pos", 1, &AdmittanceControl::changeDampAdmittanceCallback, this);
    k_adm_pos_sub_ = nh_.subscribe(manipulator_name_+"/k_adm_pos", 1, &AdmittanceControl::changeStiffAdmittanceCallback, this);
    m_adm_rot_sub_ = nh_.subscribe(manipulator_name_+"/m_adm_rot", 1, &AdmittanceControl::changeMassRotAdmittanceCallback, this);
    b_adm_rot_sub_ = nh_.subscribe(manipulator_name_+"/b_adm_rot", 1, &AdmittanceControl::changeDampRotAdmittanceCallback, this);
    k_adm_rot_sub_ = nh_.subscribe(manipulator_name_+"/k_adm_rot", 1, &AdmittanceControl::changeStiffRotAdmittanceCallback, this);

    // Load and apply parameters
    if (mode_ == "kdl")
    {
        vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(command_topic_, 1);
    }
    else
    {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>(command_topic_, 1);
    }

    tcp_pose_sub_  = nh_.subscribe(ee_pose_topic_,1,&AdmittanceControl::tcpPoseCallback,this);
    tcp_twist_sub_ = nh_.subscribe(ee_vel_topic_, 1,&AdmittanceControl::tcpTwistCallback,this);

    // Advertise service to enable admittance control
    adm_service_  = nh_.advertiseService(manipulator_name_+"/enable_admittance", &AdmittanceControl::enableAdmittance, this);
    push_service_ = nh_.advertiseService(manipulator_name_+"/enable_push_regulation", &AdmittanceControl::enablePushRegulation, this);

    // Create service client to zero the force-torque sensor
    ft_client_ = nh_.serviceClient<std_srvs::Trigger>(zero_ft_sensor_topic_);

    // Create a publisher to show the filtered msh
    wf_pub_ = nh_.advertise<geometry_msgs::Wrench>(manipulator_name_+"/filtered_wrench",1);

    // Initialize low-pass filter for the wrench
    force_filter_ = new filters::RCFilter(6, 100, 1/loop_rate_);
}

// Node params update
void AdmittanceControl::check_params()
{
    // Init joints
    if (!nh_.getParam(node_name_+"/joint_names", joint_names_))
    {
        ROS_WARN("Joint names not set, using default names.");
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}; // Default UR5 joints
    }
    n_joints_ = joint_names_.size();
    q_ = Eigen::VectorXd::Zero(n_joints_);

    // Init motion params
    std::vector<double> m_d, k_d, b_d;

    if (!nh_.getParam(node_name_+"/m_d", m_d))
    {
        ROS_WARN("Diagonal mass 'm_d' not set, using default value of 1.0.");
        m_d = {1.,1.,1.,1.,1.,1.};
    }
    if (!nh_.getParam(node_name_+"/k_d", k_d))
    {
        ROS_WARN("Diagonal spring constant 'k_d' not set, using default value of 100.0.");
        k_d = {100.,100.,100.,100.,100.,100.};
    }
    if (!nh_.getParam(node_name_+"/b_d", b_d))
    {
        ROS_WARN("Diagonal damping constant 'b_d' not set, using default value of 10.0.");
        b_d = {10.,10.,10.,10.,10.,10.};
    }

    // Admittance params
    if (!nh_.getParam(node_name_+"/dz_force", dz_force_))
    {
        ROS_WARN("Force dead zone not set, using default: 5 N.");
        dz_force_ = 5.0;
    }
    if (!nh_.getParam(node_name_+"/dz_torque", dz_torque_))
    {
        ROS_WARN("Torque dead zone not set, using default: 1 N.");
        dz_torque_ = 1.0;
    }
    if (!nh_.getParam(node_name_+"/kp_pos", kp_pos_))
    {
        ROS_WARN("Proportional gain for position compensation not set, using default: 1.0.");
        kp_pos_ = 1.0;
    }
    if (!nh_.getParam(node_name_+"/kp_rot", kp_rot_))
    {
        ROS_WARN("Proportional gain for rotation compensation not set, using default: 1.0.");
        kp_rot_ = 1.0;
    }

    // Fill matrices values
    Eigen::Matrix<double, 6, 6> M_des = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> K_des = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> B_des = Eigen::Matrix<double, 6, 6>::Zero();

    for (uint i = 0; i < 6; i++)
    {
        M_des(i, i) = m_d[i];
        K_des(i, i) = k_d[i];
        B_des(i, i) = b_d[i];
    }

    // Init interaction params
    std::vector<double> force_limit_vec;
    if (!nh_.getParam(node_name_+"/force_limit", force_limit_vec))
    {
        ROS_WARN("Force limit not set, using default values.");
        force_limit_vec = {1.0, 1.0, 1.0, 0.5, 0.5, 0.5,};
    }
    for (unsigned int k = 0; k < 6; k++) {force_limit_(k) = force_limit_vec[k];}

    if (!nh_.getParam(node_name_+"/mode", mode_))
    {
        ROS_WARN("Mode param not set, using default: kdl.");
        mode_ = "kdl";
    }
    if (mode_ == "kdl") {mode_bool_ = false;}
    else                {mode_bool_ =  true;}

    // Init control params
    if (!nh_.getParam(node_name_+"/loop_rate", loop_rate_))
    {
        ROS_WARN("Loop rate not set, using default: 500 Hz.");
        loop_rate_ = 500.0; // Default 500 Hz
    }

    // Init model params
    if (!nh_.getParam(node_name_+"/manipulator", manipulator_))
    {
        ROS_WARN("Manipulator name not set, using default: ur5.");
        manipulator_name_ = "ur5";
    }
    robot_kdl_ = new ManipulatorKDL(manipulator_);

    if (!nh_.getParam(node_name_+"/manipulator_name", manipulator_name_))
    {
        ROS_WARN("Manipulator name not set, using default: manipulator.");
        manipulator_name_ = "manipulator";
    }

    // Init command topic
    if (!nh_.getParam(node_name_+"/command_topic", command_topic_))
    {
        ROS_WARN("Command topic param not set, using default: /ur_rtde/controllers/joint_velocity_controller/command.");
        command_topic_ = "/ur_rtde/controllers/joint_velocity_controller/command";
    }
    // Init force feedback topic
    if (!nh_.getParam(node_name_+"/force_feed_topic", force_feed_topic_))
    {
        ROS_WARN("Force feedback topic param not set, using default: /ur_rtde/ft_sensor.");
        force_feed_topic_ = "/ur_rtde/ft_sensor";
    }
    // Zero force feed zero server
    if (!nh_.getParam(node_name_+"/zero_ft_topic", zero_ft_sensor_topic_))
    {
        ROS_WARN("Zero force feedback server name param not set, using default: /ur_rtde/zeroFTSensor");
        zero_ft_sensor_topic_ = "/ur_rtde/zeroFTSensor";
    }
    // Init pose feedback topic
    if (!nh_.getParam(node_name_+"/ee_pose_topic", ee_pose_topic_))
    {
        ROS_WARN("EE pose topic param not set, using default: /ur_rtde/cartesian_pose.");
        ee_pose_topic_ = "/ur_rtde/cartesian_pose";
    }
    // Init vel feedback topic
    if (!nh_.getParam(node_name_+"/ee_vel_topic", ee_vel_topic_))
    {
        ROS_WARN("EE vel topic param not set, using default: /manipulator/tcp_vel.");
        ee_vel_topic_ = "/manipulator/tcp_vel";
    }

    // Init pushing interaction params
    double kp_push, push_force_goal, safe_push_dist;
    if (!nh_.getParam(node_name_+"/kp_push", kp_push))
    {
        ROS_WARN("Proportional gain for pushing task not set, using default: 0.01 .");
        kp_push = 0.1;
    }
    if (!nh_.getParam(node_name_+"/push_force_goal", push_force_goal))
    {
        ROS_WARN("Goal force for pushing task not set, using default: 3.0 .");
        push_force_goal = 3.0;
    }
    if (!nh_.getParam(node_name_+"/safe_push_dist", safe_push_dist))
    {
        ROS_WARN("Safety distance for pushing task not set, using default: 0.25 .");
        safe_push_dist = 0.25;
    }

    // Create an instance of AdmittanceController
    adm_controller_ = new AdmittanceController(M_des, K_des, B_des,
                                               n_joints_,  1/loop_rate_,
                                               dz_force_,  dz_torque_,
                                               kp_pos_,    kp_rot_,
                                               kp_push,    push_force_goal,
                                               safe_push_dist);

}

// ------------------------------------ UTILS -------------------------- ------- //


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

// --------------------------- ROBOT STATE CALLBACKS -------------------------------

// Robot tcp pose callback
void AdmittanceControl::tcpPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
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
void AdmittanceControl::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    q_(0) = msg->position[0];
    q_(1) = msg->position[1];
    q_(2) = msg->position[2];
    q_(3) = msg->position[3];
    q_(4) = msg->position[4];
    q_(5) = msg->position[5];
}

// Robot twist callback
void AdmittanceControl::tcpTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
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
        for (uint j = 0; j < n_joints_; j++)
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
void AdmittanceControl::forceSensorCallback(const geometry_msgs::Wrench::ConstPtr &w)
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

    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force.x  = filtered_wrench(0);
    wrench_msg.force.y  = filtered_wrench(1);
    wrench_msg.force.z  = filtered_wrench(2);
    wrench_msg.torque.x = filtered_wrench(3);
    wrench_msg.torque.y = filtered_wrench(4);
    wrench_msg.torque.z = filtered_wrench(5);

    wf_pub_.publish(wrench_msg);

    return filtered_wrench;
}

// ----------------------------- SETPOINT UPDATE -----------------------------
void AdmittanceControl::admittanceXdCallback(const geometry_msgs::Pose::ConstPtr &p)
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
bool AdmittanceControl::enableAdmittance(std_srvs::SetBool::Request  &req,
                                         std_srvs::SetBool::Response &res)
{
    if (req.data)
    {
        // Enable admittance control
        adm_controller_->enableAdmittance();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;

        // Trigger the service to zero the force-torque sensor
        std_srvs::Trigger srv;
        ft_client_.call(srv);
    }
    else
    {
        // Disable admittance control
        adm_controller_->disableAdmittance();
    }
    res.success = true;
    return true;
}

// Service callback to enable or disable admittance control
bool AdmittanceControl::enablePushRegulation(std_srvs::SetBool::Request  &req,
                                             std_srvs::SetBool::Response &res)
{
    if (req.data)
    {
        // Enable admittance control
        adm_controller_->enablePush();

        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }
    else
    {
        // Disable admittance control
        adm_controller_->disablePush();
        
        // Set the desired pose equal to the current pose
        xd_ = ee_pose_;
    }
    res.success = true;
    return true;
}

// ----------------------------- MAIN FUNCTIONS -----------------------------

// Shutdown handler
void AdmittanceControl::shutdown_handler(int sig)
{
  // Show the result of the admittance control mean duration
  ROS_INFO("Mean duration of the admittance control computations: %f", mean);
  ros::Duration(1.0).sleep();

  // Shutdown ROS
  ros::shutdown();
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
    ros::Rate rate(loop_rate_);

    // ROS loop
	while (ros::ok())
	{
        // Start loop time measurement
        ros::Time start = ros::Time::now();

        // Listen to callbacks
        ros::spinOnce();

        // Execute the main computations of the admittance control
        admittance_control_main();

        // Update mean computation
        // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());
        double sample_k = ros::Time::now().toSec()-start.toSec(); k++;
        mean = 1/(static_cast<double>(k))*(sample_k+mean*static_cast<double>(k-1));

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
        std_msgs::Float64MultiArray joint_vel;
        for (uint i = 0; i < n_joints_; i++)    {joint_vel.data.push_back(dq(i));}

        // Send the command to the robot
        vel_pub_.publish(joint_vel);
    }
    else    // If "moveit" mode is enabled
    {
        // Compute the speed of as impedance control output
        Eigen::VectorXd filtered_wrench = wrenchFilter(wrench_);
        Eigen::VectorXd dx = adm_controller_->computeEESpeed(filtered_wrench,xd_,ee_pose_,dx_,dx_des_,ddx_des_);

        // Convert the vel msg as ROS msg
        geometry_msgs::Twist ee_vel;
        ee_vel.linear.x  = dx(0);
        ee_vel.linear.y  = dx(1);
        ee_vel.linear.z  = dx(2);
        ee_vel.angular.x = dx(3);
        ee_vel.angular.y = dx(4);
        ee_vel.angular.z = dx(5);

        // Send the command to the robot
        vel_pub_.publish(ee_vel);
    }
}