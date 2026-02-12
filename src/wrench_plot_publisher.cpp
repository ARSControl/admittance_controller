#include "rclcpp/rclcpp.hpp"

#include <cstdlib>
#include <thread>
#include <chrono>
#include <string>
#include <vector>

class RqtPlotLauncher : public rclcpp::Node {
public:
    RqtPlotLauncher() : Node("rqt_plot_launcher") {
        raw_wrench_topic_ = this->declare_parameter<std::string>("raw_wrench_topic", "/ur_rtde/ft_sensor");
        filtered_wrench_topic_ =
            this->declare_parameter<std::string>("filtered_wrench_topic", "/mobile_manipulator/filtered_wrench");

        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            launch_rqt_plot();
        }).detach();
    }

private:
    std::string raw_wrench_topic_;
    std::string filtered_wrench_topic_;

    std::vector<std::string> forceTopics(const std::string &base_topic) const {
        return {
            base_topic + "/force/x",
            base_topic + "/force/y",
            base_topic + "/force/z",
        };
    }

    void launch_rqt_plot() {
        std::vector<std::string> topics = forceTopics(raw_wrench_topic_);
        std::vector<std::string> filtered_topics = forceTopics(filtered_wrench_topic_);
        topics.insert(topics.end(), filtered_topics.begin(), filtered_topics.end());

        std::string cmd = "rqt_plot";
        for (const auto &topic : topics) {
            cmd += " " + topic;
        }
        cmd += " &";

        RCLCPP_INFO(this->get_logger(), "Launching force plot with raw='%s' filtered='%s'",
                    raw_wrench_topic_.c_str(), filtered_wrench_topic_.c_str());
        [[maybe_unused]] int result = std::system(cmd.c_str());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RqtPlotLauncher>());
    rclcpp::shutdown();
    return 0;
}
