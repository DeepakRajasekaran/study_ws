#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class radioNode : public rclcpp::Node 
{
public:
    radioNode() : Node("radio") 
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>("news", 10, 
                            std::bind(&radioNode::topic_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Capturing Buffer...");
    }

private:

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radioNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
