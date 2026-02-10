#include <SDL2/SDL.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

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

using Glyph = std::array<uint8_t, 7>;

const std::unordered_map<char, Glyph> kFont = {
  {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
  {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
  {'+', {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}},
  {'.', {0x00,0x00,0x00,0x00,0x00,0x06,0x06}},
  {':', {0x00,0x06,0x06,0x00,0x06,0x06,0x00}},
  {'/', {0x01,0x02,0x04,0x08,0x10,0x00,0x00}},
  {'_', {0x00,0x00,0x00,0x00,0x00,0x00,0x1F}},
  {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},
  {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
  {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
  {'3', {0x1E,0x01,0x01,0x06,0x01,0x01,0x1E}},
  {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
  {'5', {0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}},
  {'6', {0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}},
  {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
  {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
  {'9', {0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}},
  {'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},
  {'B', {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},
  {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
  {'D', {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}},
  {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
  {'F', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},
  {'G', {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}},
  {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
  {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
  {'J', {0x01,0x01,0x01,0x01,0x11,0x11,0x0E}},
  {'K', {0x11,0x12,0x14,0x18,0x14,0x12,0x11}},
  {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
  {'M', {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},
  {'N', {0x11,0x11,0x19,0x15,0x13,0x11,0x11}},
  {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
  {'P', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
  {'Q', {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}},
  {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
  {'S', {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}},
  {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
  {'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},
  {'V', {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},
  {'W', {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}},
  {'X', {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},
  {'Y', {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}},
  {'Z', {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},
};

std::string toUpper(std::string s) {
  for (char &c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
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

    joint_states_topic_ = this->declare_parameter<std::string>("topics.joint_states", "/joint_states");
    arm_cmd_topic_ = this->declare_parameter<std::string>("topics.arm_cmd_twist", "/manipulator/cmd_vel");
    arm_js_cmd_topic_ = this->declare_parameter<std::string>("topics.arm_cmd_joint", "/manipulator/js_cmd_vel");
    mobile_cmd_topic_ = this->declare_parameter<std::string>("topics.mobile_cmd_twist", "/cmd_vel");
    whole_body_cmd_topic_ = this->declare_parameter<std::string>("topics.whole_body_cmd_twist", "/mobile_manipulator/cmd_vel");

    admittance_value_ = this->declare_parameter<double>("ui.admittance_value", 20.0);
    force_reference_ = this->declare_parameter<double>("ui.force_reference", 6.0);
    max_cmd_acc_ = this->declare_parameter<double>("ui.max_cmd_acc", 10.0);
    step_value_ = this->declare_parameter<double>("ui.step_value", 0.5);

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

    auto bool_srv = [this](const std::string &name) {
      return this->create_client<std_srvs::srv::SetBool>(name);
    };

    admittance_client_ = bool_srv(admittance_service_name_);
    push_client_ = bool_srv(push_service_name_);
    wbqp_client_ = bool_srv(wbqp_service_name_);
    jacobian_client_ = bool_srv(jacobian_service_name_);
    realtime_client_ = bool_srv(joints_realtime_service_name_);

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

    status_line_ = "GUI READY";
  }

  bool initGui() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      RCLCPP_ERROR(get_logger(), "SDL init failed: %s", SDL_GetError());
      return false;
    }

    window_ = SDL_CreateWindow("ADMITTANCE CONTROL PANEL", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               kWidth, kHeight, SDL_WINDOW_SHOWN);
    if (!window_) {
      RCLCPP_ERROR(get_logger(), "SDL window failed: %s", SDL_GetError());
      SDL_Quit();
      return false;
    }

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
      render();
    }
  }

private:
  static constexpr int kWidth = 1240;
  static constexpr int kHeight = 860;

  std::mutex mutex_;

  std::string manipulator_name_;
  std::string admittance_service_name_;
  std::string push_service_name_;
  std::string wbqp_service_name_;
  std::string jacobian_service_name_;
  std::string joints_realtime_service_name_;

  std::string joint_states_topic_;
  std::string arm_cmd_topic_;
  std::string arm_js_cmd_topic_;
  std::string mobile_cmd_topic_;
  std::string whole_body_cmd_topic_;

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

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr admittance_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr push_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr wbqp_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr jacobian_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr realtime_client_;

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
  bool is_fullscreen_{false};

  double admittance_value_{20.0};
  double force_reference_{6.0};
  double max_cmd_acc_{10.0};
  double step_value_{0.5};

  std::string status_line_;

  enum class FieldId {
    AdmittanceValue = 0,
    ForceReference = 1,
    MaxCmdAcc = 2,
    StepValue = 3
  };

  void buildButtons() {
    auto addBtn = [this](const std::string &label, int x, int y, int w, int h, SDL_Color c, std::function<void()> cb) {
      buttons_.push_back(UiButton{label, SDL_Rect{x, y, w, h}, std::move(cb), c});
    };

    addBtn("ADM ON", 740, 70, 140, 34, {40, 120, 40, 255}, [this]{ callSetBool(admittance_client_, true, "ADM ON"); });
    addBtn("ADM OFF", 890, 70, 140, 34, {120, 40, 40, 255}, [this]{ callSetBool(admittance_client_, false, "ADM OFF"); });

    addBtn("PUSH ON", 740, 112, 140, 34, {40, 120, 40, 255}, [this]{ callSetBool(push_client_, true, "PUSH ON"); });
    addBtn("PUSH OFF", 890, 112, 140, 34, {120, 40, 40, 255}, [this]{ callSetBool(push_client_, false, "PUSH OFF"); });

    addBtn("WBQP ON", 740, 154, 140, 34, {40, 120, 40, 255}, [this]{ callSetBool(wbqp_client_, true, "WBQP ON"); });
    addBtn("WBQP OFF", 890, 154, 140, 34, {120, 40, 40, 255}, [this]{ callSetBool(wbqp_client_, false, "WBQP OFF"); });

    addBtn("JAC ON", 740, 196, 140, 34, {40, 120, 40, 255}, [this]{ callSetBool(jacobian_client_, true, "JAC ON"); });
    addBtn("JAC OFF", 890, 196, 140, 34, {120, 40, 40, 255}, [this]{ callSetBool(jacobian_client_, false, "JAC OFF"); });

    addBtn("RT ON", 740, 238, 140, 34, {40, 120, 40, 255}, [this]{ callSetBool(realtime_client_, true, "RT ON"); });
    addBtn("RT OFF", 890, 238, 140, 34, {120, 40, 40, 255}, [this]{ callSetBool(realtime_client_, false, "RT OFF"); });
    addBtn("FULLSCREEN", 1040, 70, 170, 34, {60, 60, 120, 255}, [this]{ toggleFullscreen(); });

    addBtn("VAL -", 740, 318, 92, 30, {70,70,120,255}, [this]{ admittance_value_ -= step_value_; });
    addBtn("VAL +", 842, 318, 92, 30, {70,70,120,255}, [this]{ admittance_value_ += step_value_; });

    addBtn("M POS", 740, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(m_adm_pos_pub_, admittance_value_, "M POS"); });
    addBtn("B POS", 840, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(b_adm_pos_pub_, admittance_value_, "B POS"); });
    addBtn("K POS", 940, 360, 92, 30, {80,80,80,255}, [this]{ publishUniform(k_adm_pos_pub_, admittance_value_, "K POS"); });

    addBtn("M ROT", 740, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(m_adm_rot_pub_, admittance_value_, "M ROT"); });
    addBtn("B ROT", 840, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(b_adm_rot_pub_, admittance_value_, "B ROT"); });
    addBtn("K ROT", 940, 398, 92, 30, {80,80,80,255}, [this]{ publishUniform(k_adm_rot_pub_, admittance_value_, "K ROT"); });

    addBtn("FREF -", 740, 476, 92, 30, {70,70,120,255}, [this]{ force_reference_ -= step_value_; });
    addBtn("FREF +", 842, 476, 92, 30, {70,70,120,255}, [this]{ force_reference_ += step_value_; });
    addBtn("SEND FREF", 944, 476, 134, 30, {50,100,130,255}, [this]{ callSetFloat(force_ref_client_, force_reference_, "SET FORCE REF"); });

    addBtn("ACC -", 740, 514, 92, 30, {70,70,120,255}, [this]{ max_cmd_acc_ = std::max(0.0, max_cmd_acc_ - step_value_); });
    addBtn("ACC +", 842, 514, 92, 30, {70,70,120,255}, [this]{ max_cmd_acc_ += step_value_; });
    addBtn("SEND ACC", 944, 514, 134, 30, {50,100,130,255}, [this]{ callSetFloat(max_acc_client_, max_cmd_acc_, "SET MAX ACC"); });

    buildTextFields();
  }

  void buildTextFields() {
    text_fields_.clear();

    auto addField = [this](const std::string &label, int x, int y, int w, int h, const std::string &text,
                           std::function<void(const std::string&)> on_apply) {
      text_fields_.push_back(UiTextField{label, SDL_Rect{x, y, w, h}, text, false, std::move(on_apply)});
    };

    addField(
      "ADM VALUE", 940, 318, 270, 30, fmt(admittance_value_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          admittance_value_ = v;
          status_line_ = "UPDATED ADM VALUE=" + fmt(admittance_value_, 2);
        } else {
          status_line_ = "INVALID ADM VALUE";
        }
      }
    );
    addField(
      "FORCE REF", 940, 442, 270, 30, fmt(force_reference_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          force_reference_ = v;
          status_line_ = "UPDATED FORCE REF=" + fmt(force_reference_, 2);
        } else {
          status_line_ = "INVALID FORCE REF";
        }
      }
    );
    addField(
      "MAX ACC", 940, 554, 270, 30, fmt(max_cmd_acc_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v)) {
          max_cmd_acc_ = std::max(0.0, v);
          status_line_ = "UPDATED MAX ACC=" + fmt(max_cmd_acc_, 2);
        } else {
          status_line_ = "INVALID MAX ACC";
        }
      }
    );
    addField(
      "STEP", 940, 590, 270, 30, fmt(step_value_, 2),
      [this](const std::string &s){
        double v;
        if (parseDouble(s, v) && v > 0.0) {
          step_value_ = v;
          status_line_ = "UPDATED STEP=" + fmt(step_value_, 2);
        } else {
          status_line_ = "INVALID STEP";
        }
      }
    );
  }

  void callSetBool(const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client, bool value, const std::string &name) {
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = value;

    if (!client->wait_for_service(std::chrono::milliseconds(80))) {
      status_line_ = "SERVICE UNAVAILABLE: " + name;
      return;
    }

    client->async_send_request(req);
    status_line_ = "REQUEST SENT: " + name;
  }

  void callSetFloat(
      const rclcpp::Client<admittance_controller::srv::SetFloat64>::SharedPtr &client,
      double value,
      const std::string &name) {
    auto req = std::make_shared<admittance_controller::srv::SetFloat64::Request>();
    req->value = value;

    if (!client->wait_for_service(std::chrono::milliseconds(80))) {
      status_line_ = "SERVICE UNAVAILABLE: " + name;
      return;
    }

    client->async_send_request(req);
    status_line_ = "REQUEST SENT: " + name + "=" + fmt(value, 2);
  }

  void publishUniform(const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr &pub, double value, const std::string &name) {
    geometry_msgs::msg::Vector3 msg;
    msg.x = value;
    msg.y = value;
    msg.z = value;
    pub->publish(msg);
    status_line_ = "PUBLISHED: " + name + "=" + fmt(value, 2);
  }

  void drawPixelText(int x, int y, const std::string &raw, int scale, SDL_Color color) {
    const std::string text = toUpper(raw);
    SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);

    int cursor_x = x;
    for (char c : text) {
      auto it = kFont.find(c);
      const Glyph *glyph = (it != kFont.end()) ? &it->second : &kFont.at(' ');
      for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
          if (((*glyph)[row] >> (4 - col)) & 0x01) {
            SDL_Rect px{cursor_x + col * scale, y + row * scale, scale, scale};
            SDL_RenderFillRect(renderer_, &px);
          }
        }
      }
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

  void drawTwistLine(int x, int y, const std::string &name, const geometry_msgs::msg::Twist &t) {
    drawPixelText(x, y, name + " L(" + fmt(t.linear.x) + "," + fmt(t.linear.y) + "," + fmt(t.linear.z) +
                        ") A(" + fmt(t.angular.x) + "," + fmt(t.angular.y) + "," + fmt(t.angular.z) + ")",
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

    {
      std::lock_guard<std::mutex> lock(mutex_);
      joint_state = last_joint_state_;
      arm_js_cmd = last_arm_js_cmd_;
      arm_cmd = last_arm_cmd_;
      mobile_cmd = last_mobile_cmd_;
      whole_cmd = last_whole_body_cmd_;
    }

    SDL_SetRenderDrawColor(renderer_, 18, 18, 22, 255);
    SDL_RenderClear(renderer_);

    SDL_Rect panel_joint{20, 20, 700, 290};
    SDL_Rect panel_cmd{20, 322, 700, 290};
    SDL_Rect panel_ctrl{730, 20, 490, 650};
    SDL_Rect panel_status{20, 624, 1200, 210};

    drawPanel(panel_joint, "JOINT STATES");
    drawPanel(panel_cmd, "COMMAND TOPICS");
    drawPanel(panel_ctrl, "CONTROL BUTTONS");
    drawPanel(panel_status, "STATUS");

    drawJointStateLine(36, 58, "JOINT POS", joint_state, false);
    drawJointStateLine(36, 92, "JOINT VEL", joint_state, true);
    drawJointStateLine(36, 126, "ARM JS CMD", arm_js_cmd, true);

    drawTwistLine(36, 360, "ARM CMD", arm_cmd);
    drawJointStateLine(36, 392, "ARM JS CMD", arm_js_cmd, true);
    drawTwistLine(36, 424, "MOBILE CMD", mobile_cmd);
    drawTwistLine(36, 456, "WHOLE CMD", whole_cmd);

    drawPixelText(742, 286, "ADM VALUE: " + fmt(admittance_value_, 2), 2, SDL_Color{220,220,180,255});
    drawPixelText(742, 442, "FORCE REF: " + fmt(force_reference_, 2), 2, SDL_Color{220,220,180,255});
    drawPixelText(742, 554, "MAX ACC: " + fmt(max_cmd_acc_, 2) + "  STEP:" + fmt(step_value_,2), 2, SDL_Color{220,220,180,255});

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
    drawPixelText(36, 728, "CLICK A FIELD TO EDIT, ENTER TO APPLY", 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(36, 762, "PLUS/MINUS BUTTONS STILL WORK", 2, SDL_Color{200, 200, 200, 255});
    drawPixelText(36, 796, is_fullscreen_ ? "VIEW: FULLSCREEN" : "VIEW: WINDOWED", 2, SDL_Color{200, 200, 200, 255});

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

  void syncFieldTexts() {
    if (text_fields_.size() < 4) {
      return;
    }
    text_fields_[static_cast<size_t>(FieldId::AdmittanceValue)].text = fmt(admittance_value_, 2);
    text_fields_[static_cast<size_t>(FieldId::ForceReference)].text = fmt(force_reference_, 2);
    text_fields_[static_cast<size_t>(FieldId::MaxCmdAcc)].text = fmt(max_cmd_acc_, 2);
    text_fields_[static_cast<size_t>(FieldId::StepValue)].text = fmt(step_value_, 2);
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

  void toggleFullscreen() {
    if (!window_) {
      return;
    }
    is_fullscreen_ = !is_fullscreen_;
    const Uint32 flag = is_fullscreen_ ? static_cast<Uint32>(SDL_WINDOW_FULLSCREEN_DESKTOP) : static_cast<Uint32>(0);
    if (SDL_SetWindowFullscreen(window_, flag) != 0) {
      is_fullscreen_ = !is_fullscreen_;
      status_line_ = std::string("FULLSCREEN TOGGLE FAILED: ") + SDL_GetError();
    } else {
      status_line_ = is_fullscreen_ ? "FULLSCREEN ENABLED" : "WINDOWED ENABLED";
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
