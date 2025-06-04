#include "rclcpp/rclcpp.hpp"
#include <cstdlib>
#include <thread>
#include <chrono>

class RqtPlotLauncher : public rclcpp::Node {
public:
    RqtPlotLauncher() : Node("rqt_plot_launcher") {
        // Launch in separate threads so both rqt_plot instances open
        std::thread(&RqtPlotLauncher::launch_rqt_plot, this,
            "/ur_rtde/ft_sensor").detach();

        std::thread(&RqtPlotLauncher::launch_rqt_plot, this,
            "/manipulator/filtered_wrench").detach();
    }

private:
    void launch_rqt_plot(const std::string& topic) {
        // Wait a moment for ROS graph to populate
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::string base = topic;
        std::string cmd = "rqt_plot "
            + base + "/force/x "
            + base + "/force/y "
            + base + "/force/z "
            + base + "/torque/x "
            + base + "/torque/y "
            + base + "/torque/z &";

        (void)std::system(cmd.c_str());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RqtPlotLauncher>());
    rclcpp::shutdown();
    return 0;
}
