#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/srv/RectangleParameters.hpp"
#include <cstdlib>

class clientNode : public rclcpp::Node 
{
public:
    clientNode() : Node("client_node") 
    {
        srand(time(0));
        RCLCPP_INFO(this->get_logger(), "Client_Sumserver has been initiated...");
        /* timer_name */ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&/* ClassName */::/* timer_callback_name */, this));
        
    }

private:
    std::vector<std::thread> threads_;
    void client_request()
    {
        RCLCPP_INFO(this->get_logger(), "Timer event");
        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<clientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
