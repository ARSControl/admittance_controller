#include <cmath>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JoyModeCommandNode : public rclcpp::Node
{
public:
    JoyModeCommandNode()
    : Node("joy_mode_command_node")
    {
        declareAndLoadParameters();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic_,
            10,
            std::bind(&JoyModeCommandNode::joyCallback, this, std::placeholders::_1)
        );

        arm_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(arm_twist_topic_, 10);
        arm_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(arm_joint_topic_, 10);
        mobile_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(mobile_twist_topic_, 10);
        whole_body_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(whole_body_twist_topic_, 10);

        // Fixed publish rate: 20 Hz (50 ms period)
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JoyModeCommandNode::publishCommands, this)
        );

        RCLCPP_INFO(this->get_logger(), "joy_mode_command_node started. Listening on '%s'.", joy_topic_.c_str());
    }

private:
    enum class Mode
    {
        STOP,
        ARM,
        MOBILE,
        WHOLE_BODY
    };

    enum class ArmControlMode
    {
        CARTESIAN_TWIST,
        JOINT_STATE
    };

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_twist_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mobile_twist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr whole_body_twist_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Topics
    std::string joy_topic_;
    std::string arm_twist_topic_;
    std::string arm_joint_topic_;
    std::string mobile_twist_topic_;
    std::string whole_body_twist_topic_;

    // Joint command metadata
    std::vector<std::string> joint_names_;

    // Mode button mappings
    int button_mode_arm_;
    int button_mode_mobile_;
    int button_mode_whole_body_;
    int button_mode_stop_;

    // Arm sub-mode buttons
    int button_arm_joint_mode_l1_;
    int button_arm_joint_mode_r1_;
    int button_arm_pair_34_fallback_;
    int button_arm_pair_56_fallback_;

    // Axes mappings
    int axis_arm_linear_x_;
    int axis_arm_linear_y_;
    int axis_arm_linear_z_;
    int axis_arm_joint_left_;
    int axis_arm_joint_right_;
    int axis_arm_pair_34_trigger_;
    int axis_arm_pair_56_trigger_;

    int axis_mobile_linear_x_;
    int axis_mobile_linear_y_;
    int axis_mobile_angular_z_;

    int axis_whole_body_linear_x_;
    int axis_whole_body_linear_y_;
    int axis_whole_body_linear_z_;
    int axis_whole_body_rotation_trigger_;

    // Scales and deadzone
    double scale_arm_linear_x_;
    double scale_arm_linear_y_;
    double scale_arm_linear_z_;
    double scale_arm_angular_x_;
    double scale_arm_angular_y_;
    double scale_arm_angular_z_;
    double scale_arm_joint_;

    double scale_mobile_linear_x_;
    double scale_mobile_linear_y_;
    double scale_mobile_angular_z_;

    double scale_whole_body_linear_x_;
    double scale_whole_body_linear_y_;
    double scale_whole_body_linear_z_;
    double scale_whole_body_angular_x_;
    double scale_whole_body_angular_y_;
    double scale_whole_body_angular_z_;

    double axis_deadzone_;
    double trigger_pressed_threshold_;

    // Shared state between /joy callback and timer callback
    mutable std::mutex mutex_;
    sensor_msgs::msg::Joy last_joy_msg_;
    std::vector<int32_t> prev_buttons_;
    bool has_joy_msg_{false};
    Mode mode_{Mode::STOP};
    bool stop_zero_pending_{true};
    ArmControlMode arm_control_mode_{ArmControlMode::CARTESIAN_TWIST};

    void declareAndLoadParameters()
    {
        joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");

        arm_twist_topic_ = this->declare_parameter<std::string>("topics.arm_twist", "/manipulator/cmd_vel");
        arm_joint_topic_ = this->declare_parameter<std::string>("topics.arm_joint", "/manipulator/js_cmd_vel");
        mobile_twist_topic_ = this->declare_parameter<std::string>("topics.mobile_twist", "/neo/cmd_vel");
        whole_body_twist_topic_ = this->declare_parameter<std::string>("topics.whole_body_twist", "/mobile_manipulator/cmd_vel");

        joint_names_ = this->declare_parameter<std::vector<std::string>>(
            "joint_names",
            std::vector<std::string>{
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            }
        );

        button_mode_arm_ = this->declare_parameter<int>("buttons.mode_arm", 0);                // A
        button_mode_mobile_ = this->declare_parameter<int>("buttons.mode_mobile", 2);          // X
        button_mode_whole_body_ = this->declare_parameter<int>("buttons.mode_whole_body", 3);  // Y
        button_mode_stop_ = this->declare_parameter<int>("buttons.mode_stop", 1);              // B

        const int legacy_arm_joint_mode = this->declare_parameter<int>("buttons.arm_joint_mode", 9);
        button_arm_joint_mode_l1_ = this->declare_parameter<int>("buttons.arm_joint_mode_l1", legacy_arm_joint_mode);
        button_arm_joint_mode_r1_ = this->declare_parameter<int>("buttons.arm_joint_mode_r1", 10);
        button_arm_pair_34_fallback_ = this->declare_parameter<int>("buttons.arm_pair_34", -1);
        button_arm_pair_56_fallback_ = this->declare_parameter<int>("buttons.arm_pair_56", -1);

        axis_arm_linear_x_ = this->declare_parameter<int>("axes.arm_linear_x", 1);
        axis_arm_linear_y_ = this->declare_parameter<int>("axes.arm_linear_y", 0);
        axis_arm_linear_z_ = this->declare_parameter<int>("axes.arm_linear_z", 3);
        axis_arm_joint_left_ = this->declare_parameter<int>("axes.arm_joint_left", 0);
        axis_arm_joint_right_ = this->declare_parameter<int>("axes.arm_joint_right", 3);
        axis_arm_pair_34_trigger_ = this->declare_parameter<int>("axes.arm_pair_34_trigger", 4);
        axis_arm_pair_56_trigger_ = this->declare_parameter<int>("axes.arm_pair_56_trigger", 5);

        axis_mobile_linear_x_ = this->declare_parameter<int>("axes.mobile_linear_x", 1);
        axis_mobile_linear_y_ = this->declare_parameter<int>("axes.mobile_linear_y", 0);
        axis_mobile_angular_z_ = this->declare_parameter<int>("axes.mobile_angular_z", 3);

        axis_whole_body_linear_x_ = this->declare_parameter<int>("axes.whole_body_linear_x", 1);
        axis_whole_body_linear_y_ = this->declare_parameter<int>("axes.whole_body_linear_y", 0);
        axis_whole_body_linear_z_ = this->declare_parameter<int>("axes.whole_body_linear_z", 3);
        axis_whole_body_rotation_trigger_ = this->declare_parameter<int>(
            "axes.whole_body_rotation_trigger",
            axis_arm_pair_56_trigger_
        );

        scale_arm_linear_x_ = this->declare_parameter<double>("scales.arm_linear_x", 0.3);
        scale_arm_linear_y_ = this->declare_parameter<double>("scales.arm_linear_y", 0.3);
        scale_arm_linear_z_ = this->declare_parameter<double>("scales.arm_linear_z", 0.2);
        scale_arm_angular_x_ = this->declare_parameter<double>("scales.arm_angular_x", 0.3);
        scale_arm_angular_y_ = this->declare_parameter<double>("scales.arm_angular_y", 0.3);
        scale_arm_angular_z_ = this->declare_parameter<double>("scales.arm_angular_z", 0.2);
        scale_arm_joint_ = this->declare_parameter<double>("scales.arm_joint", 0.6);

        scale_mobile_linear_x_ = this->declare_parameter<double>("scales.mobile_linear_x", 0.4);
        scale_mobile_linear_y_ = this->declare_parameter<double>("scales.mobile_linear_y", 0.4);
        scale_mobile_angular_z_ = this->declare_parameter<double>("scales.mobile_angular_z", 0.6);

        scale_whole_body_linear_x_ = this->declare_parameter<double>("scales.whole_body_linear_x", 0.4);
        scale_whole_body_linear_y_ = this->declare_parameter<double>("scales.whole_body_linear_y", 0.4);
        scale_whole_body_linear_z_ = this->declare_parameter<double>("scales.whole_body_linear_z", 0.4);
        scale_whole_body_angular_x_ = this->declare_parameter<double>("scales.whole_body_angular_x", 0.6);
        scale_whole_body_angular_y_ = this->declare_parameter<double>("scales.whole_body_angular_y", 0.6);
        scale_whole_body_angular_z_ = this->declare_parameter<double>("scales.whole_body_angular_z", 0.6);

        axis_deadzone_ = this->declare_parameter<double>("axis_deadzone", 0.05);
        trigger_pressed_threshold_ = this->declare_parameter<double>("trigger_pressed_threshold", -0.9);
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (prev_buttons_.size() != msg->buttons.size()) {
            prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
        }

        // Mode selection is event-based (rising edge), as requested.
        if (isRisingEdge(*msg, button_mode_stop_)) {
            setModeLocked(Mode::STOP);
        } else if (isRisingEdge(*msg, button_mode_arm_)) {
            setModeLocked(Mode::ARM);
        } else if (isRisingEdge(*msg, button_mode_mobile_)) {
            setModeLocked(Mode::MOBILE);
        } else if (isRisingEdge(*msg, button_mode_whole_body_)) {
            setModeLocked(Mode::WHOLE_BODY);
        }

        // Arm sub-mode selection is event-based:
        // R1 -> joint control mode, L1 -> cartesian cmd_vel mode.
        if (isRisingEdge(*msg, button_arm_joint_mode_r1_)) {
            arm_control_mode_ = ArmControlMode::JOINT_STATE;
            RCLCPP_INFO(this->get_logger(), "Arm sub-mode changed to JOINT_STATE");
        } else if (isRisingEdge(*msg, button_arm_joint_mode_l1_)) {
            arm_control_mode_ = ArmControlMode::CARTESIAN_TWIST;
            RCLCPP_INFO(this->get_logger(), "Arm sub-mode changed to CARTESIAN_TWIST");
        }

        last_joy_msg_ = *msg;
        prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());
        has_joy_msg_ = true;
    }

    void publishCommands()
    {
        sensor_msgs::msg::Joy joy_msg;
        Mode mode = Mode::STOP;
        bool has_joy_msg = false;
        bool stop_zero_pending = false;
        ArmControlMode arm_control_mode = ArmControlMode::CARTESIAN_TWIST;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            mode = mode_;
            arm_control_mode = arm_control_mode_;
            if (has_joy_msg_) {
                joy_msg = last_joy_msg_;
                has_joy_msg = true;
            }
            stop_zero_pending = stop_zero_pending_;
            if (mode_ == Mode::STOP && stop_zero_pending_) {
                stop_zero_pending_ = false;
            }
        }

        if (mode == Mode::STOP) {
            if (stop_zero_pending) {
                // In STOP mode publish a single zero command burst, then remain silent.
                publishZeroAll();
            }
            return;
        }

        if (!has_joy_msg) {
            return;
        }

        switch (mode) {
            case Mode::ARM:
                if (arm_control_mode == ArmControlMode::JOINT_STATE) {
                    arm_joint_pub_->publish(buildArmJointCmd(joy_msg));
                } else {
                    arm_twist_pub_->publish(buildArmTwistCmd(joy_msg));
                }
                break;

            case Mode::MOBILE:
                mobile_twist_pub_->publish(buildMobileTwistCmd(joy_msg));
                break;

            case Mode::WHOLE_BODY:
                whole_body_twist_pub_->publish(buildWholeBodyTwistCmd(joy_msg));
                break;

            case Mode::STOP:
            default:
                break;
        }
    }

    void publishZeroAll()
    {
        arm_twist_pub_->publish(geometry_msgs::msg::Twist());
        arm_joint_pub_->publish(buildZeroJointCmd());
        mobile_twist_pub_->publish(geometry_msgs::msg::Twist());
        whole_body_twist_pub_->publish(geometry_msgs::msg::Twist());
    }

    sensor_msgs::msg::JointState buildZeroJointCmd() const
    {
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = this->now();
        cmd.name = joint_names_;
        cmd.velocity = std::vector<double>(joint_names_.size(), 0.0);
        return cmd;
    }

    geometry_msgs::msg::Twist buildArmTwistCmd(const sensor_msgs::msg::Joy &joy) const
    {
        geometry_msgs::msg::Twist cmd;
        const double x = axisWithDeadzone(joy, axis_arm_linear_x_);
        const double y = axisWithDeadzone(joy, axis_arm_linear_y_);
        const double z = axisWithDeadzone(joy, axis_arm_linear_z_);

        // While R2 trigger is pressed, cartesian arm mode switches from linear to angular twist.
        if (isTriggerPressed(joy, axis_arm_pair_56_trigger_)) {
            cmd.angular.x = x * scale_arm_angular_x_;
            cmd.angular.y = y * scale_arm_angular_y_;
            cmd.angular.z = z * scale_arm_angular_z_;
        } else {
            cmd.linear.x = x * scale_arm_linear_x_;
            cmd.linear.y = y * scale_arm_linear_y_;
            cmd.linear.z = z * scale_arm_linear_z_;
        }
        return cmd;
    }

    sensor_msgs::msg::JointState buildArmJointCmd(const sensor_msgs::msg::Joy &joy) const
    {
        sensor_msgs::msg::JointState cmd = buildZeroJointCmd();

        const double left_axis = axisWithDeadzone(joy, axis_arm_joint_left_) * scale_arm_joint_;
        const double right_axis = axisWithDeadzone(joy, axis_arm_joint_right_) * scale_arm_joint_;

        // Optional pair expansion, enabled while modifiers are pressed.
        const bool pair_34_pressed =
            isTriggerPressed(joy, axis_arm_pair_34_trigger_) ||
            isButtonActive(joy, button_arm_pair_34_fallback_);
        const bool pair_56_pressed =
            isTriggerPressed(joy, axis_arm_pair_56_trigger_) ||
            isButtonActive(joy, button_arm_pair_56_fallback_);

        // Joints 1-2 are commanded only when no trigger modifier is active.
        if (!pair_34_pressed && !pair_56_pressed) {
            if (cmd.velocity.size() > 0U) { cmd.velocity[0] = left_axis; }
            if (cmd.velocity.size() > 1U) { cmd.velocity[1] = right_axis; }
        }

        if (pair_34_pressed) {
            if (cmd.velocity.size() > 2U) { cmd.velocity[2] = left_axis; }
            if (cmd.velocity.size() > 3U) { cmd.velocity[3] = right_axis; }
        }

        if (pair_56_pressed) {
            if (cmd.velocity.size() > 4U) { cmd.velocity[4] = left_axis; }
            if (cmd.velocity.size() > 5U) { cmd.velocity[5] = right_axis; }
        }

        return cmd;
    }

    geometry_msgs::msg::Twist buildMobileTwistCmd(const sensor_msgs::msg::Joy &joy) const
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = axisWithDeadzone(joy, axis_mobile_linear_x_) * scale_mobile_linear_x_;
        cmd.linear.y = axisWithDeadzone(joy, axis_mobile_linear_y_) * scale_mobile_linear_y_;
        cmd.angular.z = axisWithDeadzone(joy, axis_mobile_angular_z_) * scale_mobile_angular_z_;
        return cmd;
    }

    geometry_msgs::msg::Twist buildWholeBodyTwistCmd(const sensor_msgs::msg::Joy &joy) const
    {
        geometry_msgs::msg::Twist cmd;
        const double x = axisWithDeadzone(joy, axis_whole_body_linear_x_);
        const double y = axisWithDeadzone(joy, axis_whole_body_linear_y_);
        const double z = axisWithDeadzone(joy, axis_whole_body_linear_z_);

        // Whole-body mode mirrors arm cartesian behavior:
        // default linear XYZ, hold R2 to command angular XYZ with same three axes.
        if (isTriggerPressed(joy, axis_whole_body_rotation_trigger_)) {
            cmd.angular.x = x * scale_whole_body_angular_x_;
            cmd.angular.y = y * scale_whole_body_angular_y_;
            cmd.angular.z = z * scale_whole_body_angular_z_;
        } else {
            cmd.linear.x = x * scale_whole_body_linear_x_;
            cmd.linear.y = y * scale_whole_body_linear_y_;
            cmd.linear.z = z * scale_whole_body_linear_z_;
        }
        return cmd;
    }

    double axisWithDeadzone(const sensor_msgs::msg::Joy &joy, int axis_idx) const
    {
        if (axis_idx < 0 || static_cast<size_t>(axis_idx) >= joy.axes.size()) {
            return 0.0;
        }

        const double value = joy.axes[static_cast<size_t>(axis_idx)];
        return (std::abs(value) < axis_deadzone_) ? 0.0 : value;
    }

    bool isButtonActive(const sensor_msgs::msg::Joy &joy, int button_idx) const
    {
        if (button_idx < 0 || static_cast<size_t>(button_idx) >= joy.buttons.size()) {
            return false;
        }

        return joy.buttons[static_cast<size_t>(button_idx)] != 0;
    }

    bool isTriggerPressed(const sensor_msgs::msg::Joy &joy, int axis_idx) const
    {
        if (axis_idx < 0 || static_cast<size_t>(axis_idx) >= joy.axes.size()) {
            return false;
        }

        return joy.axes[static_cast<size_t>(axis_idx)] < trigger_pressed_threshold_;
    }

    bool isRisingEdge(const sensor_msgs::msg::Joy &joy, int button_idx) const
    {
        if (button_idx < 0 || static_cast<size_t>(button_idx) >= joy.buttons.size()) {
            return false;
        }

        const int32_t current = joy.buttons[static_cast<size_t>(button_idx)];
        int32_t previous = 0;

        if (static_cast<size_t>(button_idx) < prev_buttons_.size()) {
            previous = prev_buttons_[static_cast<size_t>(button_idx)];
        }

        return (previous == 0) && (current != 0);
    }

    void setModeLocked(Mode new_mode)
    {
        // STOP can be re-triggered while already in STOP: every B press must publish zero once.
        if (new_mode == Mode::STOP) {
            stop_zero_pending_ = true;
        }

        if (mode_ == new_mode) {
            return;
        }

        mode_ = new_mode;
        RCLCPP_INFO(this->get_logger(), "Mode changed to %s", modeToString(mode_).c_str());
    }

    static std::string modeToString(Mode mode)
    {
        switch (mode) {
            case Mode::ARM: return "ARM";
            case Mode::MOBILE: return "MOBILE";
            case Mode::WHOLE_BODY: return "WHOLE_BODY";
            case Mode::STOP:
            default: return "STOP";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyModeCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
