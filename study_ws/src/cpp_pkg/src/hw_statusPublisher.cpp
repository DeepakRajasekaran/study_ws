#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node 
{
public:
    HardwareStatusPublisherNode() : Node("HardwareStatusPublisher") 
    {
        RCLCPP_INFO(this->get_logger(), "hw_status Publisher has been initiated..");
        hw_status = this->create_publisher<irobot_interfaces::msg::HardwareStatus>("hw_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HardwareStatusPublisherNode::publishHardwareStatus, this));
    }
private:

    void publishHardwareStatus()
    {
        auto msg = irobot_interfaces::msg::HardwareStatus();
        msg.temperature = 40;
        msg.motor_status = true;
        msg.debug_message = "Everything is working fine!";
        RCLCPP_INFO(this->get_logger(), "Publishing message...");
        hw_status->publish(msg);
    }

    rclcpp::Publisher<irobot_interfaces::msg::HardwareStatus>::SharedPtr hw_status;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
