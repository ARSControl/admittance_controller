#include <SDL2/SDL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <neo_msgs2/msg/emergency_stop_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ur_rtde_controller/srv/roboti_q_gripper_control.hpp>

#include <admittance_controller/srv/set_float64.hpp>

namespace {

struct UiButton {
  std::string label;
  SDL_Rect rect{};
  std::function<void()> on_click;
  SDL_Color color{70, 70, 70, 255};
};

struct UiTextField {
  std::string label;
  SDL_Rect rect{};
  std::string text;
  bool active{false};
  std::function<void(const std::string&)> on_apply;
};

struct Glyph
{
  std::array<uint8_t, 7> rows{};
};

const std::unordered_map<char, Glyph> &Font()
{
  static const std::unordered_map<char, Glyph> glyphs{
      {' ', {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}},
      {'!', {{0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04}}},
      {'"', {{0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00}}},
      {'#', {{0x0A, 0x1F, 0x0A, 0x0A, 0x1F, 0x0A, 0x00}}},
      {'$', {{0x04, 0x0F, 0x14, 0x0E, 0x05, 0x1E, 0x04}}},
      {'%', {{0x10, 0x11, 0x02, 0x04, 0x08, 0x11, 0x01}}},
      {'&', {{0x0C, 0x12, 0x14, 0x08, 0x15, 0x12, 0x0D}}},
      {'\'',{{0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00}}},
      {'(', {{0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02}}},
      {')', {{0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08}}},
      {'*', {{0x00, 0x04, 0x15, 0x0E, 0x15, 0x04, 0x00}}},
      {'+', {{0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}}},
      {',', {{0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08}}},
      {'-', {{0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00}}},
      {'.', {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04}}},
      {'/', {{0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00}}},
      {'0', {{0x1E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x1E}}},
      {'1', {{0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x1F}}},
      {'2', {{0x1E, 0x11, 0x01, 0x06, 0x18, 0x10, 0x1F}}},
      {'3', {{0x1E, 0x11, 0x02, 0x06, 0x02, 0x11, 0x1E}}},
      {'4', {{0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}}},
      {'5', {{0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x1E}}},
      {'6', {{0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x1E}}},
      {'7', {{0x1F, 0x11, 0x02, 0x04, 0x08, 0x08, 0x08}}},
      {'8', {{0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}}},
      {'9', {{0x1E, 0x11, 0x11, 0x1F, 0x01, 0x02, 0x1C}}},
      {':', {{0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00}}},
      {';', {{0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x08}}},
      {'<', {{0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02}}},
      {'=', {{0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00}}},
      {'>', {{0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08}}},
      {'?', {{0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}}},
      {'@', {{0x0E, 0x11, 0x17, 0x15, 0x17, 0x10, 0x0E}}},
      {'A', {{0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}},
      {'B', {{0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}}},
      {'C', {{0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}}},
      {'D', {{0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C}}},
      {'E', {{0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}}},
      {'F', {{0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}}},
      {'G', {{0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}}},
      {'H', {{0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}}},
      {'I', {{0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F}}},
      {'J', {{0x1F, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C}}},
      {'K', {{0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}}},
      {'L', {{0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}}},
      {'M', {{0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}}},
      {'N', {{0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}}},
      {'O', {{0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}},
      {'P', {{0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}}},
      {'Q', {{0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}}},
      {'R', {{0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}}},
      {'S', {{0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}}},
      {'T', {{0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}}},
      {'U', {{0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}}},
      {'V', {{0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}}},
      {'W', {{0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}}},
      {'X', {{0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}}},
      {'Y', {{0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}}},
      {'Z', {{0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}}},
      {'[', {{0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E}}},
      {'\\', {{0x10, 0x08, 0x04, 0x02, 0x01, 0x00, 0x00}}},
      {']', {{0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E}}},
      {'^', {{0x04, 0x0A, 0x11, 0x00, 0x00, 0x00, 0x00}}},
      {'_', {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F}}},
      {'`', {{0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00}}},
      {'a', {{0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F}}},
      {'b', {{0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x1E}}},
      {'c', {{0x00, 0x00, 0x0E, 0x11, 0x10, 0x11, 0x0E}}},
      {'d', {{0x01, 0x01, 0x0F, 0x11, 0x11, 0x11, 0x0F}}},
      {'e', {{0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0E}}},
      {'f', {{0x06, 0x08, 0x1E, 0x08, 0x08, 0x08, 0x08}}},
      {'g', {{0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x1E}}},
      {'h', {{0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x11}}},
      {'i', {{0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E}}},
      {'j', {{0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C}}},
      {'k', {{0x10, 0x10, 0x12, 0x14, 0x18, 0x14, 0x12}}},
      {'l', {{0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}}},
      {'m', {{0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15}}},
      {'n', {{0x00, 0x00, 0x1E, 0x11, 0x11, 0x11, 0x11}}},
      {'o', {{0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E}}},
      {'p', {{0x00, 0x00, 0x1E, 0x11, 0x11, 0x1E, 0x10}}},
      {'q', {{0x00, 0x00, 0x0F, 0x11, 0x11, 0x0F, 0x01}}},
      {'r', {{0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10}}},
      {'s', {{0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E}}},
      {'t', {{0x08, 0x08, 0x1E, 0x08, 0x08, 0x08, 0x06}}},
      {'u', {{0x00, 0x00, 0x11, 0x11, 0x11, 0x11, 0x0F}}},
      {'v', {{0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04}}},
      {'w', {{0x00, 0x00, 0x11, 0x11, 0x15, 0x1B, 0x11}}},
      {'x', {{0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11}}},
      {'y', {{0x00, 0x00, 0x11, 0x0A, 0x04, 0x08, 0x10}}},
      {'z', {{0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F}}},
      {'{', {{0x02, 0x04, 0x04, 0x08, 0x04, 0x04, 0x02}}},
      {'|', {{0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}}},
      {'}', {{0x08, 0x04, 0x04, 0x02, 0x04, 0x04, 0x08}}},
      {'~', {{0x00, 0x00, 0x09, 0x15, 0x12, 0x00, 0x00}}},
  };
  return glyphs;
}

const std::unordered_map<char32_t, Glyph> &FontIT()
{
  static const std::unordered_map<char32_t, Glyph> glyphs{
      {U'£', {{0x0E, 0x08, 0x1E, 0x08, 0x08, 0x1F, 0x00}}},
      {U'€', {{0x0E, 0x10, 0x1E, 0x10, 0x1E, 0x10, 0x0E}}},
      {U'°', {{0x06, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00}}},
      {U'§', {{0x0E, 0x10, 0x0E, 0x01, 0x0E, 0x10, 0x0E}}},
      {U'à', {{0x08, 0x04, 0x0E, 0x01, 0x0F, 0x11, 0x0F}}},
      {U'è', {{0x08, 0x04, 0x0E, 0x11, 0x1F, 0x10, 0x0E}}},
      {U'ì', {{0x08, 0x04, 0x0C, 0x04, 0x04, 0x04, 0x0E}}},
      {U'ò', {{0x08, 0x04, 0x0E, 0x11, 0x11, 0x11, 0x0E}}},
      {U'ù', {{0x08, 0x04, 0x11, 0x11, 0x11, 0x11, 0x0F}}},
      {U'À', {{0x04, 0x02, 0x0E, 0x11, 0x1F, 0x11, 0x11}}},
      {U'È', {{0x04, 0x02, 0x1F, 0x10, 0x1E, 0x10, 0x1F}}},
      {U'Ì', {{0x04, 0x02, 0x1F, 0x04, 0x04, 0x04, 0x1F}}},
      {U'Ò', {{0x04, 0x02, 0x0E, 0x11, 0x11, 0x11, 0x0E}}},
      {U'Ù', {{0x04, 0x02, 0x11, 0x11, 0x11, 0x11, 0x0E}}},
  };
  return glyphs;
}

void DrawGlyph(SDL_Renderer *renderer, int x, int y, char32_t ch, SDL_Color color, int scale)
{
  const auto &font = Font();
  const auto &fontIT = FontIT();

  Glyph pattern{};
  if (auto it = font.find(static_cast<char>(ch)); it != font.end())
  {
    pattern = it->second;
  }
  else if (auto iu = fontIT.find(ch); iu != fontIT.end())
  {
    pattern = iu->second;
  }
  else
  {
    pattern = font.at(' ');
  }

  SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
  const int step = std::max(1, scale);
  for (int row = 0; row < 7; ++row)
  {
    for (int col = 0; col < 5; ++col)
    {
      if ((pattern.rows[static_cast<size_t>(row)] >> (4 - col)) & 0x1)
      {
        if (step == 1)
        {
          SDL_RenderDrawPoint(renderer, x + col, y + row);
        }
        else
        {
          SDL_Rect rect{x + col * step, y + row * step, step, step};
          SDL_RenderFillRect(renderer, &rect);
        }
      }
    }
  }
}

std::string fmt(double v, int p = 3) {
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(p);
  oss << v;
  return oss.str();
}

}  // namespace

class AdmittanceGuiNode : public rclcpp::Node {
public:
  AdmittanceGuiNode() : Node("admittance_gui_node") {
    manipulator_name_ = this->declare_parameter<std::string>("manipulator_name", "manipulator");
    admittance_service_name_ = this->declare_parameter<std::string>(
      "services.admittance_enable", manipulator_name_ + "/enable_admittance");
    push_service_name_ = this->declare_parameter<std::string>(
      "services.push_enable", manipulator_name_ + "/enable_push_regulation");
    wbqp_service_name_ = this->declare_parameter<std::string>("services.wbqp_enable", "/wbqp_controller/enable_qp");
    jacobian_service_name_ = this->declare_parameter<std::string>(
      "services.jacobian_realtime_enable", manipulator_name_ + "/jacobian_control_setter");
    joints_realtime_service_name_ = this->declare_parameter<std::string>(
      "services.joints_realtime_enable", manipulator_name_ + "/joints_real_time_setter");
    mobile_emergency_stop_service_name_ = this->declare_parameter<std::string>(
      "services.mobile_emergency_stop", "/mobile_platform/emergency_stop");

    joint_states_topic_ = this->declare_parameter<std::string>("topics.joint_states", "/joint_states");
    arm_cmd_topic_ = this->declare_parameter<std::string>("topics.arm_cmd_twist", "/manipulator/cmd_vel");
    arm_js_cmd_topic_ = this->declare_parameter<std::string>("topics.arm_cmd_joint", "/manipulator/js_cmd_vel");
    mobile_cmd_topic_ = this->declare_parameter<std::string>("topics.mobile_cmd_twist", "/cmd_vel");
    whole_body_cmd_topic_ = this->declare_parameter<std::string>("topics.whole_body_cmd_twist", "/mobile_manipulator/cmd_vel");
    emergency_topic_ = this->declare_parameter<std::string>("topics.emergency_state", "/emergency_stop_state");
    battery_topic_ = this->declare_parameter<std::string>("topics.battery_status", "/battery_status");

    arm_ip_ = this->declare_parameter<std::string>("network.arm_ip", "192.168.2.10");
    mobile_ip_ = this->declare_parameter<std::string>("network.mobile_ip", "192.168.2.50");
    ping_period_ms_ = this->declare_parameter<int>("network.ping_period_ms", 1000);

    this->declare_parameter<double>("ui.admittance_value");
    this->declare_parameter<double>("ui.force_reference");
    this->declare_parameter<double>("ui.max_cmd_acc");
    this->declare_parameter<double>("ui.step_value");
    this->declare_parameter<int>("ui.gripper_position");
    this->declare_parameter<int>("ui.gripper_speed");
    this->declare_parameter<int>("ui.gripper_force");
    gripper_service_name_ = this->declare_parameter<std::string>(
      "services.gripper_control", "/ur_rtde_controller/robotiq_gripper_control");

    admittance_value_ = this->get_parameter("ui.admittance_value").as_double();
    force_reference_ = this->get_parameter("ui.force_reference").as_double();
    max_cmd_acc_ = this->get_parameter("ui.max_cmd_acc").as_double();
    step_value_ = this->get_parameter("ui.step_value").as_double();
    gripper_position_ = this->get_parameter("ui.gripper_position").as_int();
    gripper_speed_ = this->get_parameter("ui.gripper_speed").as_int();
    gripper_force_ = this->get_parameter("ui.gripper_force").as_int();

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, 10, [this](sensor_msgs::msg::JointState::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        last_joint_state_ = *msg;
      });

    arm_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      arm_cmd_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        last_arm_cmd_ = *msg;
      });
    arm_js_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_js_cmd_topic_, 10, [this](sensor_msgs::msg::JointState::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        last_arm_js_cmd_ = *msg;
      });
    mobile_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      mobile_cmd_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        last_mobile_cmd_ = *msg;
      });
    whole_body_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      whole_body_cmd_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        last_whole_body_cmd_ = *msg;
      });

    emergency_sub_ = create_subscription<neo_msgs2::msg::EmergencyStopState>(
      emergency_topic_, 10, [this](neo_msgs2::msg::EmergencyStopState::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        emergency_button_stop_ = msg->emergency_button_stop;
        scanner_stop_ = msg->scanner_stop;
        has_emergency_state_ = true;
      });

    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_, 10, [this](sensor_msgs::msg::BatteryState::SharedPtr msg){
        std::lock_guard<std::mutex> lock(mutex_);
        battery_voltage_ = msg->voltage;
        battery_current_ = msg->current;
        battery_temperature_ = msg->temperature;
        battery_percentage_ = msg->percentage;
        has_battery_state_ = true;
      });

    auto bool_srv = [this](const std::string &name) {
      return this->create_client<std_srvs::srv::SetBool>(name);
    };

    admittance_client_ = bool_srv(admittance_service_name_);
    push_client_ = bool_srv(push_service_name_);
    wbqp_client_ = bool_srv(wbqp_service_name_);
    jacobian_client_ = bool_srv(jacobian_service_name_);
    realtime_client_ = bool_srv(joints_realtime_service_name_);
    mobile_emergency_stop_client_ = bool_srv(mobile_emergency_stop_service_name_);
    gripper_client_ = this->create_client<ur_rtde_controller::srv::RobotiQGripperControl>(gripper_service_name_);

    force_ref_client_ = this->create_client<admittance_controller::srv::SetFloat64>(
      manipulator_name_ + "/set_push_force_reference");
    max_acc_client_ = this->create_client<admittance_controller::srv::SetFloat64>(
      manipulator_name_ + "/set_max_cmd_acceleration");

    m_adm_pos_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_pos", 10);
    b_adm_pos_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_pos", 10);
    k_adm_pos_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_pos", 10);
    m_adm_rot_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_rot", 10);
    b_adm_rot_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_rot", 10);
    k_adm_rot_pub_ = create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_rot", 10);

    status_line_ = "Gui ready";
    last_ping_check_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(std::max(100, ping_period_ms_));
  }

  bool initGui() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      RCLCPP_ERROR(get_logger(), "SDL init failed: %s", SDL_GetError());
      return false;
    }

    window_ = SDL_CreateWindow(
      "ADMITTANCE CONTROL PANEL",
      SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED,
      kWidth,
      kHeight,
      SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    if (!window_) {
      RCLCPP_ERROR(get_logger(), "SDL window failed: %s", SDL_GetError());
      SDL_Quit();
      return false;
    }
    SDL_SetWindowResizable(window_, SDL_TRUE);
    SDL_SetWindowMinimumSize(window_, 1120, 700);

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer_) {
      RCLCPP_ERROR(get_logger(), "SDL renderer failed: %s", SDL_GetError());
      SDL_DestroyWindow(window_);
      SDL_Quit();
      return false;
    }

    buildButtons();
    syncFieldTexts();
    SDL_StartTextInput();
    return true;
  }

  void shutdownGui() {
    SDL_StopTextInput();
    if (renderer_) {
      SDL_DestroyRenderer(renderer_);
      renderer_ = nullptr;
    }
    if (window_) {
      SDL_DestroyWindow(window_);
      window_ = nullptr;
    }
    SDL_Quit();
  }

  void run() {
    bool running = true;
    while (running && rclcpp::ok()) {
      SDL_Event event;
      while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
          running = false;
          break;
        }
        if (event.type == SDL_TEXTINPUT) {
          std::string chunk(event.text.text);
          if (isTextInputAllowed(chunk)) {
            handleTextInput(chunk);
          }
        }
        if (event.type == SDL_KEYDOWN) {
          if (event.key.keysym.sym == SDLK_BACKSPACE) {
            handleBackspace();
          } else if (event.key.keysym.sym == SDLK_RETURN || event.key.keysym.sym == SDLK_KP_ENTER) {
            handleEnter();
          }
        }
        if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
          handleMouse(event.button.x, event.button.y);
        }
      }

      rclcpp::spin_some(shared_from_this());
      updatePingStatus();
      render();
    }
  }

private:
  static constexpr int kWidth = 1560;
  static constexpr int kHeight = 980;

  std::mutex mutex_;

  std::string manipulator_name_;
  std::string admittance_service_name_;
  std::string push_service_name_;
  std::string wbqp_service_name_;
  std::string jacobian_service_name_;
  std::string joints_realtime_service_name_;
  std::string mobile_emergency_stop_service_name_;
  std::string gripper_service_name_;

  std::string joint_states_topic_;
  std::string arm_cmd_topic_;
  std::string arm_js_cmd_topic_;
  std::string mobile_cmd_topic_;
  std::string whole_body_cmd_topic_;
  std::string emergency_topic_;
  std::string battery_topic_;

  std::string arm_ip_;
  std::string mobile_ip_;
  int ping_period_ms_{1000};

  sensor_msgs::msg::JointState last_joint_state_;
  geometry_msgs::msg::Twist last_arm_cmd_;
  sensor_msgs::msg::JointState last_arm_js_cmd_;
  geometry_msgs::msg::Twist last_mobile_cmd_;
  geometry_msgs::msg::Twist last_whole_body_cmd_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_js_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mobile_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr whole_body_cmd_sub_;
  rclcpp::Subscription<neo_msgs2::msg::EmergencyStopState>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr admittance_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr push_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr wbqp_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr jacobian_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mobile_emergency_stop_client_;
  rclcpp::Client<ur_rtde_controller::srv::RobotiQGripperControl>::SharedPtr gripper_client_;

  rclcpp::Client<admittance_controller::srv::SetFloat64>::SharedPtr force_ref_client_;
  rclcpp::Client<admittance_controller::srv::SetFloat64>::SharedPtr max_acc_client_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_adm_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr b_adm_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr k_adm_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_adm_rot_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr b_adm_rot_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr k_adm_rot_pub_;

  SDL_Window *window_{nullptr};
  SDL_Renderer *renderer_{nullptr};
  std::vector<UiButton> buttons_;
  std::vector<UiTextField> text_fields_;

  double admittance_value_{0.0};
  double force_reference_{0.0};
  double max_cmd_acc_{0.0};
  double step_value_{0.0};
  int gripper_position_{100};
  int gripper_speed_{100};
  int gripper_force_{100};

  bool arm_ping_ok_{false};
  bool mobile_robot_ping_ok_{false};
  double arm_ping_avg_5s_{0.0};
  double mobile_robot_ping_avg_5s_{0.0};
  std::chrono::steady_clock::time_point last_ping_check_{};
  std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> arm_ping_samples_;
  std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> mobile_robot_ping_samples_;

  bool has_emergency_state_{false};
  bool emergency_button_stop_{false};
  bool scanner_stop_{false};

  bool has_battery_state_{false};
  double battery_voltage_{0.0};
  double battery_current_{0.0};
  double battery_temperature_{0.0};
  double battery_percentage_{0.0};

  std::string status_line_;

  bool mode_adm_on_{false};
  bool mode_push_on_{false};
  bool mode_wbqp_on_{false};
  bool mode_arm_jac_on_{false};
  bool mode_arm_jts_on_{false};
  bool mode_emergency_on_{true};

  enum class FieldId {
    AdmittanceValue = 0,
    ForceReference = 1,
    MaxCmdAcc = 2,
    StepValue = 3,
    GripperPosition = 4,
    GripperSpeed = 5,
    GripperForce = 6
  };

  void buildButtons() {
    auto addBtn = [this](const std::string &label, int x, int y, int w, int h, SDL_Color c, std::function<void()> cb) {
      buttons_.push_back(UiButton{label, SDL_Rect{x, y, w, h}, std::move(cb), c});
    };

    addBtn("Adm on", 810, 70, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(admittance_client_, true, "Adm on")) { mode_adm_on_ = true; }
    });
    addBtn("Adm off", 960, 70, 140, 34, {120, 40, 40, 255}, [this]{
      if (callSetBool(admittance_client_, false, "Adm off")) { mode_adm_on_ = false; }
    });

    addBtn("Push on", 810, 112, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(push_client_, true, "Push on")) { mode_push_on_ = true; }
    });
    addBtn("Push off", 960, 112, 140, 34, {120, 40, 40, 255}, [this]{
      if (callSetBool(push_client_, false, "Push off")) { mode_push_on_ = false; }
    });

    addBtn("Wbqp on", 810, 154, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(wbqp_client_, true, "Wbqp on")) { mode_wbqp_on_ = true; }
    });
    addBtn("Wbqp off", 960, 154, 140, 34, {120, 40, 40, 255}, [this]{
      if (callSetBool(wbqp_client_, false, "Wbqp off")) { mode_wbqp_on_ = false; }
    });

    addBtn("Arm Jac on", 810, 196, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(jacobian_client_, true, "Arm jac on")) { mode_arm_jac_on_ = true; }
    });
    addBtn("Arm Jac off", 960, 196, 140, 34, {120, 40, 40, 255}, [this]{
      if (callSetBool(jacobian_client_, false, "Arm jac off")) { mode_arm_jac_on_ = false; }
    });

    addBtn("Arm Jts on", 810, 238, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(realtime_client_, true, "Arm jts on")) { mode_arm_jts_on_ = true; }
    });
    addBtn("Arm Jts off", 960, 238, 140, 34, {120, 40, 40, 255}, [this]{
      if (callSetBool(realtime_client_, false, "Arm jts off")) { mode_arm_jts_on_ = false; }
    });

    addBtn("Emerg on", 1110, 238, 140, 34, {120, 60, 40, 255}, [this]{
      if (callSetBool(mobile_emergency_stop_client_, true, "Emerg on")) { mode_emergency_on_ = true; }
    });
    addBtn("Emerg off", 1260, 238, 140, 34, {40, 120, 40, 255}, [this]{
      if (callSetBool(mobile_emergency_stop_client_, false, "Emerg off")) { mode_emergency_on_ = false; }
    });

    addBtn("Val -", 810, 318, 92, 30, {70,70,120,255}, [this]{ admittance_value_ -= step_value_; });
    addBtn("Val +", 912, 318, 92, 30, {70,70,120,255}, [this]{ admittance_value_ += step_value_; });

    addBtn("M pos", 810, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(m_adm_pos_pub_, admittance_value_, "M pos"); });
    addBtn("B pos", 910, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(b_adm_pos_pub_, admittance_value_, "B pos"); });
    addBtn("K pos", 1010, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(k_adm_pos_pub_, admittance_value_, "K pos"); });

    addBtn("M rot", 810, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(m_adm_rot_pub_, admittance_value_, "M rot"); });
    addBtn("B rot", 910, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(b_adm_rot_pub_, admittance_value_, "B rot"); });
    addBtn("K rot", 1010, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(k_adm_rot_pub_, admittance_value_, "K rot"); });

    addBtn("Fref -", 810, 476, 92, 30, {70,70,120,255}, [this]{ force_reference_ -= step_value_; });
    addBtn("Fref +", 912, 476, 92, 30, {70,70,120,255}, [this]{ force_reference_ += step_value_; });
    addBtn("Send fref", 1014, 476, 134, 30, {50,100,130,255}, [this]{ callSetFloat(force_ref_client_, force_reference_, "Set force ref"); });

    addBtn("Acc -", 810, 514, 92, 30, {70,70,120,255}, [this]{ max_cmd_acc_ = std::max(0.0, max_cmd_acc_ - step_value_); });
    addBtn("Acc +", 912, 514, 92, 30, {70,70,120,255}, [this]{ max_cmd_acc_ += step_value_; });
    addBtn("Send acc", 1014, 514, 134, 30, {50,100,130,255}, [this]{ callSetFloat(max_acc_client_, max_cmd_acc_, "Set max acc"); });

    addBtn("Grip open", 1110, 318, 130, 30, {40,120,40,255}, [this]{
      gripper_position_ = ur_rtde_controller::srv::RobotiQGripperControl::Request::GRIPPER_OPENED;
      callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper open");
    });
    addBtn("Grip close", 1250, 318, 130, 30, {120,40,40,255}, [this]{
      gripper_position_ = 0;
      callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper close");
    });
    addBtn("Grip send", 1390, 318, 130, 30, {50,100,130,255}, [this]{
      callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper set");
    });

    buildTextFields();
  }

  void buildTextFields() {
    text_fields_.clear();

    auto addField = [this](const std::string &label, int x, int y, int w, int h, const std::string &text,
                           std::function<void(const std::string&)> on_apply) {
      text_fields_.push_back(UiTextField{label, SDL_Rect{x, y, w, h}, text, false, std::move(on_apply)});
    };

    addField(
      "Adm value", 1260, 318, 260, 30, fmt(admittance_value_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          admittance_value_ = v;
          status_line_ = "Updated adm value=" + fmt(admittance_value_, 2);
        } else {
          status_line_ = "Invalid adm value";
        }
      }
    );
    addField(
      "Force ref", 1260, 442, 260, 30, fmt(force_reference_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          force_reference_ = v;
          status_line_ = "Updated force ref=" + fmt(force_reference_, 2);
        } else {
          status_line_ = "Invalid force ref";
        }
      }
    );
    addField(
      "Max acc", 1260, 546, 260, 30, fmt(max_cmd_acc_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          max_cmd_acc_ = std::max(0.0, v);
          status_line_ = "Updated max acc=" + fmt(max_cmd_acc_, 2);
        } else {
          status_line_ = "Invalid max acc";
        }
      }
    );
    addField(
      "Step", 1260, 590, 260, 30, fmt(step_value_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v) && v > 0.0) {
          step_value_ = v;
          status_line_ = "Updated step=" + fmt(step_value_, 2);
        } else {
          status_line_ = "Invalid step";
        }
      }
    );
    addField(
      "Grip pos", 1110, 360, 130, 30, std::to_string(gripper_position_),
      [this](const std::string &s){
        int v;
        if (parseIntRange(s, 0, 100, v)) {
          gripper_position_ = v;
          callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper set");
        } else {
          status_line_ = "Invalid grip pos";
        }
      }
    );
    addField(
      "Grip speed", 1250, 360, 130, 30, std::to_string(gripper_speed_),
      [this](const std::string &s){
        int v;
        if (parseIntRange(s, 0, 100, v)) {
          gripper_speed_ = v;
          callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper set");
        } else {
          status_line_ = "Invalid grip speed";
        }
      }
    );
    addField(
      "Grip force", 1390, 360, 130, 30, std::to_string(gripper_force_),
      [this](const std::string &s){
        int v;
        if (parseIntRange(s, 0, 100, v)) {
          gripper_force_ = v;
          callGripper(gripper_position_, gripper_speed_, gripper_force_, "Gripper set");
        } else {
          status_line_ = "Invalid grip force";
        }
      }
    );
  }

  bool callSetBool(const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client, bool value, const std::string &name) {
    if (!client) {
      status_line_ = "Service client missing: " + name;
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = value;

    if (!client->wait_for_service(std::chrono::milliseconds(80))) {
      status_line_ = "Service unavailable: " + name;
      return false;
    }

    client->async_send_request(req);
    status_line_ = "Request sent: " + name;
    return true;
  }

  void callSetFloat(
      const rclcpp::Client<admittance_controller::srv::SetFloat64>::SharedPtr &client,
      double value,
      const std::string &name) {
    auto req = std::make_shared<admittance_controller::srv::SetFloat64::Request>();
    req->value = value;

    if (!client->wait_for_service(std::chrono::milliseconds(80))) {
      status_line_ = "Service unavailable: " + name;
      return;
    }

    client->async_send_request(req);
    status_line_ = "Request sent: " + name + "=" + fmt(value, 2);
  }

  void callGripper(int position, int speed, int force, const std::string &name) {
    if (!gripper_client_) {
      status_line_ = "Service client missing: " + name;
      return;
    }

    auto req = std::make_shared<ur_rtde_controller::srv::RobotiQGripperControl::Request>();
    req->position = std::clamp(position, 0, 100);
    req->speed = std::clamp(speed, 0, 100);
    req->force = std::clamp(force, 0, 100);

    if (!gripper_client_->wait_for_service(std::chrono::milliseconds(120))) {
      status_line_ = "Service unavailable: " + name;
      return;
    }

    const auto future = gripper_client_->async_send_request(
      req,
      [this](rclcpp::Client<ur_rtde_controller::srv::RobotiQGripperControl>::SharedFuture response_future) {
        const auto &resp = response_future.get();
        status_line_ = std::string("Gripper resp: ") +
          (resp->success ? "ok" : "fail") +
          ", status=" + std::to_string(resp->status);
      });
    (void)future;
    status_line_ = "Request sent: " + name;
  }

  void publishUniform(const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr &pub, double value, const std::string &name) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = value;
    msg.y = value;
    msg.z = value;
    pub->publish(msg);
    status_line_ = "Published: " + name + "=" + fmt(value, 2);
  }

  void drawPixelText(int x, int y, const std::string &raw, int scale, SDL_Color color) {
    const std::string text = raw;
    int cursor_x = x;
    for (unsigned char c : text) {
      DrawGlyph(renderer_, cursor_x, y, static_cast<char32_t>(c), color, scale);
      cursor_x += 6 * scale;
    }
  }

  void drawPanel(const SDL_Rect &r, const std::string &title) {
    SDL_SetRenderDrawColor(renderer_, 30, 30, 36, 255);
    SDL_RenderFillRect(renderer_, &r);
    SDL_SetRenderDrawColor(renderer_, 90, 90, 100, 255);
    SDL_RenderDrawRect(renderer_, &r);
    drawPixelText(r.x + 8, r.y + 8, title, 2, SDL_Color{210, 210, 220, 255});
  }

  void drawModeIndicator(int x, int y, const std::string &label, bool active) {
    SDL_Rect box{x, y, 210, 28};
    const SDL_Color fill = active ? SDL_Color{35, 110, 50, 255} : SDL_Color{55, 55, 60, 255};
    SDL_SetRenderDrawColor(renderer_, fill.r, fill.g, fill.b, fill.a);
    SDL_RenderFillRect(renderer_, &box);
    SDL_SetRenderDrawColor(renderer_, 120, 120, 120, 255);
    SDL_RenderDrawRect(renderer_, &box);
    drawPixelText(
      x + 6,
      y + 7,
      label + ": " + (active ? "on" : "off"),
      2,
      active ? SDL_Color{160, 255, 160, 255} : SDL_Color{190, 190, 190, 255});
  }

  void drawTwistLine(int x, int y, const std::string &name, const geometry_msgs::msg::Twist &t) {
    drawPixelText(x, y, name + " l(" + fmt(t.linear.x) + "," + fmt(t.linear.y) + "," + fmt(t.linear.z) +
                        ") a(" + fmt(t.angular.x) + "," + fmt(t.angular.y) + "," + fmt(t.angular.z) + ")",
                 2, SDL_Color{220, 220, 220, 255});
  }

  void drawJointStateLine(int x, int y, const std::string &name, const sensor_msgs::msg::JointState &js, bool velocity_only = false) {
    std::string line = name + " ";
    const auto &vals = velocity_only ? js.velocity : js.position;
    const size_t n = std::min<size_t>(vals.size(), 6);
    line += "[";
    for (size_t i = 0; i < n; ++i) {
      line += fmt(vals[i]);
      if (i + 1 < n) {
        line += ",";
      }
    }
    line += "]";
    drawPixelText(x, y, line, 2, SDL_Color{220, 220, 220, 255});
  }

  void render() {
    sensor_msgs::msg::JointState joint_state;
    sensor_msgs::msg::JointState arm_js_cmd;
    geometry_msgs::msg::Twist arm_cmd;
    geometry_msgs::msg::Twist mobile_cmd;
    geometry_msgs::msg::Twist whole_cmd;
    double arm_ping_avg_5s = 0.0;
    double mobile_robot_ping_avg_5s = 0.0;
    bool has_emergency = false;
    bool button_stop = false;
    bool scanner_stop = false;
    bool has_battery = false;
    double battery_voltage = 0.0;
    double battery_current = 0.0;
    double battery_temperature = 0.0;
    double battery_percentage = 0.0;
    bool mode_adm_on = false;
    bool mode_push_on = false;
    bool mode_wbqp_on = false;
    bool mode_arm_jac_on = false;
    bool mode_arm_jts_on = false;
    bool mode_emergency_on = true;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      joint_state = last_joint_state_;
      arm_js_cmd = last_arm_js_cmd_;
      arm_cmd = last_arm_cmd_;
      mobile_cmd = last_mobile_cmd_;
      whole_cmd = last_whole_body_cmd_;
      arm_ping_avg_5s = arm_ping_avg_5s_;
      mobile_robot_ping_avg_5s = mobile_robot_ping_avg_5s_;
      has_emergency = has_emergency_state_;
      button_stop = emergency_button_stop_;
      scanner_stop = scanner_stop_;
      has_battery = has_battery_state_;
      battery_voltage = battery_voltage_;
      battery_current = battery_current_;
      battery_temperature = battery_temperature_;
      battery_percentage = battery_percentage_;
      mode_adm_on = mode_adm_on_;
      mode_push_on = mode_push_on_;
      mode_wbqp_on = mode_wbqp_on_;
      mode_arm_jac_on = mode_arm_jac_on_;
      mode_arm_jts_on = mode_arm_jts_on_;
      mode_emergency_on = mode_emergency_on_;
    }

    SDL_SetRenderDrawColor(renderer_, 18, 18, 22, 255);
    SDL_RenderClear(renderer_);

    SDL_Rect panel_joint{20, 20, 760, 290};
    SDL_Rect panel_cmd{20, 322, 760, 290};
    SDL_Rect panel_status{20, 624, 760, 336};
    SDL_Rect panel_ctrl{790, 20, 750, 620};
    SDL_Rect panel_gripper{1098, 282, 432, 124};
    SDL_Rect panel_ping{790, 650, 365, 310};
    SDL_Rect panel_mobile_state{1175, 650, 365, 310};

    drawPanel(panel_joint, "JOINT STATES");
    drawPanel(panel_cmd, "COMMAND TOPICS");
    drawPanel(panel_ctrl, "CONTROL BUTTONS");
    drawPanel(panel_gripper, "GRIPPER CONTROL");
    drawPanel(panel_status, "STATUS");
    drawPanel(panel_ping, "PING STATUS");
    drawPanel(panel_mobile_state, "MOBILE STATE");

    drawJointStateLine(36, 58, "Joint pos", joint_state, false);
    drawJointStateLine(36, 92, "Joint vel", joint_state, true);
    drawJointStateLine(36, 126, "Arm js cmd", arm_js_cmd, true);

    drawTwistLine(36, 360, "Arm cmd", arm_cmd);
    drawJointStateLine(36, 392, "Arm js cmd", arm_js_cmd, true);
    drawTwistLine(36, 424, "Mobile cmd", mobile_cmd);
    drawTwistLine(36, 456, "Whole cmd", whole_cmd);

    drawPixelText(812, 286, "Adm value: " + fmt(admittance_value_, 2), 2, SDL_Color{220,220,180,255});
    drawPixelText(812, 442, "Force ref: " + fmt(force_reference_, 2), 2, SDL_Color{220,220,180,255});
    drawPixelText(812, 526, "Max acc: " + fmt(max_cmd_acc_, 2), 2, SDL_Color{220,220,180,255});
    drawPixelText(812, 570, "Step: " + fmt(step_value_,2), 2, SDL_Color{220,220,180,255});
    drawModeIndicator(1260, 70, "Admittance", mode_adm_on);
    drawModeIndicator(1260, 104, "Pushing", mode_push_on);
    drawModeIndicator(1260, 138, "Wbqp", mode_wbqp_on);
    drawModeIndicator(1260, 172, "Arm jac", mode_arm_jac_on);
    drawModeIndicator(1260, 206, "Arm jts", mode_arm_jts_on);
    drawModeIndicator(1260, 240, "Emergency", mode_emergency_on);

    for (const auto &b : buttons_) {
      SDL_SetRenderDrawColor(renderer_, b.color.r, b.color.g, b.color.b, b.color.a);
      SDL_RenderFillRect(renderer_, &b.rect);
      SDL_SetRenderDrawColor(renderer_, 170, 170, 170, 255);
      SDL_RenderDrawRect(renderer_, &b.rect);
      drawPixelText(b.rect.x + 8, b.rect.y + 9, b.label, 2, SDL_Color{235, 235, 235, 255});
    }

    drawTextFields();

    drawPixelText(36, 662, "STATUS", 2, SDL_Color{180, 200, 220, 255});
    drawPixelText(36, 694, status_line_, 2, SDL_Color{240, 220, 130, 255});
    drawPixelText(36, 728, "Click a field to edit.", 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(36, 762, "Press enter to apply.", 2, SDL_Color{200, 200, 200, 255});

    drawPixelText(806, 722, "Arm avg 5s: " + fmt(arm_ping_avg_5s * 100.0, 1) + "%", 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(806, 772, "Mobile robot avg 5s: " + fmt(mobile_robot_ping_avg_5s * 100.0, 1) + "%", 2, SDL_Color{200, 200, 200, 255});

    drawPixelText(1191, 688, std::string("Button stop: ") + (has_emergency ? (button_stop ? "true" : "false") : "n/a"), 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(1191, 722, std::string("Scanner stop: ") + (has_emergency ? (scanner_stop ? "true" : "false") : "n/a"), 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(1191, 772, std::string("Voltage: ") + (has_battery ? fmt(battery_voltage, 3) : "n/a"), 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(1191, 806, std::string("Current: ") + (has_battery ? fmt(battery_current, 3) : "n/a"), 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(1191, 840, std::string("Temperature: ") + (has_battery ? fmt(battery_temperature, 3) : "n/a"), 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(1191, 874, std::string("Percentage: ") + (has_battery ? fmt(battery_percentage * 100.0, 1) + "%" : "n/a"), 2, SDL_Color{200, 200, 200, 255});

    SDL_RenderPresent(renderer_);
  }

  void handleMouse(int x, int y) {
    bool clicked_field = false;
    for (size_t i = 0; i < text_fields_.size(); ++i) {
      auto &f = text_fields_[i];
      const bool inside = x >= f.rect.x && x <= (f.rect.x + f.rect.w) && y >= f.rect.y && y <= (f.rect.y + f.rect.h);
      f.active = inside;
      clicked_field = clicked_field || inside;
    }

    if (clicked_field) {
      return;
    }

    for (auto &b : buttons_) {
      if (x >= b.rect.x && x <= (b.rect.x + b.rect.w) && y >= b.rect.y && y <= (b.rect.y + b.rect.h)) {
        b.on_click();
        syncFieldTexts();
        break;
      }
    }
  }

  void handleTextInput(const std::string &txt) {
    UiTextField *active = getActiveField();
    if (!active) {
      return;
    }
    active->text += txt;
  }

  void handleBackspace() {
    UiTextField *active = getActiveField();
    if (!active || active->text.empty()) {
      return;
    }
    active->text.pop_back();
  }

  void handleEnter() {
    UiTextField *active = getActiveField();
    if (!active) {
      return;
    }
    active->on_apply(active->text);
    syncFieldTexts();
  }

  UiTextField* getActiveField() {
    for (auto &f : text_fields_) {
      if (f.active) {
        return &f;
      }
    }
    return nullptr;
  }

  static bool parseDouble(const std::string &s, double &out) {
    try {
      size_t idx = 0;
      out = std::stod(s, &idx);
      return idx == s.size();
    } catch (...) {
      return false;
    }
  }

  static bool parseIntRange(const std::string &s, int min_v, int max_v, int &out) {
    try {
      size_t idx = 0;
      const int v = std::stoi(s, &idx);
      if (idx != s.size()) {
        return false;
      }
      if (v < min_v || v > max_v) {
        return false;
      }
      out = v;
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool isTextInputAllowed(const std::string &chunk) {
    if (chunk.empty()) {
      return false;
    }
    for (char c : chunk) {
      const bool ok = (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
      if (!ok) {
        return false;
      }
    }
    return true;
  }

  static bool isSafeIpToken(const std::string &ip) {
    if (ip.empty()) {
      return false;
    }
    for (char c : ip) {
      const bool ok =
        (c >= '0' && c <= '9') ||
        (c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        c == '.' || c == ':' || c == '-';
      if (!ok) {
        return false;
      }
    }
    return true;
  }

  static bool pingHost(const std::string &ip) {
    if (!isSafeIpToken(ip)) {
      return false;
    }
    const std::string cmd = "ping -c 1 -W 1 " + ip + " > /dev/null 2>&1";
    return std::system(cmd.c_str()) == 0;
  }

  static void updatePingWindow(
      std::deque<std::pair<std::chrono::steady_clock::time_point, bool>> &samples,
      const std::chrono::steady_clock::time_point &now,
      bool value,
      double &avg_out) {
    samples.emplace_back(now, value);
    const auto window_start = now - std::chrono::seconds(5);
    while (!samples.empty() && samples.front().first < window_start) {
      samples.pop_front();
    }

    if (samples.empty()) {
      avg_out = 0.0;
      return;
    }

    size_t good = 0U;
    for (const auto &s : samples) {
      if (s.second) {
        ++good;
      }
    }
    avg_out = static_cast<double>(good) / static_cast<double>(samples.size());
  }

  void updatePingStatus() {
    const auto now = std::chrono::steady_clock::now();
    const int ping_period_ms = 1000;
    (void)ping_period_ms_;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ping_check_).count() < ping_period_ms) {
      return;
    }

    const bool arm_ok = pingHost(arm_ip_);
    const bool mobile_robot_ok = pingHost(mobile_ip_);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      arm_ping_ok_ = arm_ok;
      mobile_robot_ping_ok_ = mobile_robot_ok;
      updatePingWindow(arm_ping_samples_, now, arm_ok, arm_ping_avg_5s_);
      updatePingWindow(mobile_robot_ping_samples_, now, mobile_robot_ok, mobile_robot_ping_avg_5s_);
      last_ping_check_ = now;
    }
  }

  void syncFieldTexts() {
    if (text_fields_.size() < 7) {
      return;
    }
    text_fields_[static_cast<size_t>(FieldId::AdmittanceValue)].text = fmt(admittance_value_, 2);
    text_fields_[static_cast<size_t>(FieldId::ForceReference)].text = fmt(force_reference_, 2);
    text_fields_[static_cast<size_t>(FieldId::MaxCmdAcc)].text = fmt(max_cmd_acc_, 2);
    text_fields_[static_cast<size_t>(FieldId::StepValue)].text = fmt(step_value_, 2);
    text_fields_[static_cast<size_t>(FieldId::GripperPosition)].text = std::to_string(gripper_position_);
    text_fields_[static_cast<size_t>(FieldId::GripperSpeed)].text = std::to_string(gripper_speed_);
    text_fields_[static_cast<size_t>(FieldId::GripperForce)].text = std::to_string(gripper_force_);
  }

  void drawTextFields() {
    for (const auto &f : text_fields_) {
      SDL_SetRenderDrawColor(renderer_, f.active ? 70 : 45, f.active ? 90 : 45, 60, 255);
      SDL_RenderFillRect(renderer_, &f.rect);
      SDL_SetRenderDrawColor(renderer_, f.active ? 220 : 140, f.active ? 220 : 140, f.active ? 120 : 140, 255);
      SDL_RenderDrawRect(renderer_, &f.rect);
      drawPixelText(f.rect.x + 6, f.rect.y + 8, f.label + ": " + f.text, 2, SDL_Color{235, 235, 235, 255});
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AdmittanceGuiNode>();
  if (!node->initGui()) {
    rclcpp::shutdown();
    return 1;
  }

  node->run();
  node->shutdownGui();

  rclcpp::shutdown();
  return 0;
}
