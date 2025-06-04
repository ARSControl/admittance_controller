#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <cstdlib>
#include <thread>
#include <chrono>
#include <string>

class RqtPlotLauncher : public rclcpp::Node {
public:
    RqtPlotLauncher() : Node("rqt_plot_launcher") {
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            launch_rqt_with_perspective("admittance_controller", "config/plots/WrenchPlot.perspective");
        }).detach();
    }

private:
    void launch_rqt_with_perspective(const std::string& package_name, const std::string& relative_path) {
        try {
            std::string package_share = ament_index_cpp::get_package_share_directory(package_name);
            std::string full_path = package_share + "/" + relative_path;

            std::string cmd = "rqt --perspective-file " + full_path + " &";
            [[maybe_unused]] int result = std::system(cmd.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find package or launch rqt: %s", e.what());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RqtPlotLauncher>());
    rclcpp::shutdown();
    return 0;
}
