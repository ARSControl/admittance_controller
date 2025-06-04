#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <iostream>
#include <string>

class MenuClientNode : public rclcpp::Node
{
public:
    MenuClientNode()
    : Node("admittance_menu_node")
    {
        this->declare_parameter("manipulator_name", "manipulator");
        manipulator_name_ = this->get_parameter("manipulator_name").as_string();

        // Service clients
        adm_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name_ + "/enable_admittance");
        push_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name_ + "/enable_push_regulation");

        // Parameter publishers
        m_adm_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_pos", 1);
        b_adm_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_pos", 1);
        k_adm_pos_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_pos", 1);
        m_adm_rot_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/m_adm_rot", 1);
        b_adm_rot_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/b_adm_rot", 1);
        k_adm_rot_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(manipulator_name_ + "/k_adm_rot", 1);

        // Pose publisher
        xd_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(manipulator_name_ + "/adm_xd", 1);
    }

    void runMenu()
    {
        while (rclcpp::ok()) {
            printMenu();

            int choice;
            std::cin >> choice;

            if (choice == 0) {
                std::cout << "Exiting menu.\n";
                break;
            }

            bool enable;
            switch (choice) {
                case 1:
                    std::cout << "Enable (1) or Disable (0) admittance: ";
                    std::cin >> enable;
                    callService(adm_client_, enable, "Admittance");
                    break;
                case 2:
                    std::cout << "Enable (1) or Disable (0) push regulation: ";
                    std::cin >> enable;
                    callService(push_client_, enable, "Push Regulation");
                    break;
                case 3: publishVectorUniform("Translational Mass", m_adm_pos_pub_); break;
                case 4: publishVectorUniform("Translational Damping", b_adm_pos_pub_); break;
                case 5: publishVectorUniform("Translational Stiffness", k_adm_pos_pub_); break;
                case 6: publishVectorUniform("Rotational Mass", m_adm_rot_pub_); break;
                case 7: publishVectorUniform("Rotational Damping", b_adm_rot_pub_); break;
                case 8: publishVectorUniform("Rotational Stiffness", k_adm_rot_pub_); break;
                case 9: publishDesiredPose(); break;
                default:
                    std::cout << "Invalid choice.\n";
            }
        }
    }

private:
    std::string manipulator_name_;

    // Service clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr adm_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr push_client_;

    // Admittance parameter publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_adm_pos_pub_, b_adm_pos_pub_, k_adm_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_adm_rot_pub_, b_adm_rot_pub_, k_adm_rot_pub_;

    // Desired pose publisher
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr xd_pub_;

    void printMenu()
    {
        std::cout << "\n===== Admittance Control Menu =====\n";
        std::cout << "1. Enable/Disable Admittance Control\n";
        std::cout << "2. Enable/Disable Push Regulation\n";
        std::cout << "3. Set Translational Mass\n";
        std::cout << "4. Set Translational Damping\n";
        std::cout << "5. Set Translational Stiffness\n";
        std::cout << "6. Set Rotational Mass\n";
        std::cout << "7. Set Rotational Damping\n";
        std::cout << "8. Set Rotational Stiffness\n";
        std::cout << "9. Set Desired Cartesian Pose (xd)\n";
        std::cout << "0. Exit\n";
        std::cout << "Select option: ";
    }

    void callService(const rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr &client, bool enable, const std::string &label)
    {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            std::cerr << label << " service not available.\n";
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enable;

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            std::cout << label << " response: " << response->message << "\n";
        } else {
            std::cerr << "Service call failed.\n";
        }
    }

    void publishVectorUniform(const std::string &label,
                              const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr &pub)
    {
        double value;
        std::cout << "Enter " << label << " value (applied to x/y/z): ";
        std::cin >> value;

        geometry_msgs::msg::Vector3 msg;
        msg.x = msg.y = msg.z = value;
        pub->publish(msg);

        std::cout << label << " published as (" << value << ", " << value << ", " << value << ")\n";
    }

    void publishDesiredPose()
    {
        geometry_msgs::msg::Pose msg;

        std::cout << "Enter position (x y z): ";
        std::cin >> msg.position.x >> msg.position.y >> msg.position.z;

        std::cout << "Enter orientation (x y z w): ";
        std::cin >> msg.orientation.x
                 >> msg.orientation.y
                 >> msg.orientation.z
                 >> msg.orientation.w;

        xd_pub_->publish(msg);
        std::cout << "Published desired pose.\n";
    }
};

// -----------------------
// MAIN
// -----------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MenuClientNode>();
    node->runMenu();
    rclcpp::shutdown();
    return 0;
}
