#include "rclcpp/rclcpp.hpp"
//#include "example_interfaces/msg/string.hpp"
#include "std_msgs/msg/string.hpp"


class news_station : public rclcpp::Node 
{
public:
    news_station() : Node("news"), robot_name_("chappie :)")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("news", 10);
        timer_ =  this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&news_station::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Broadcasting Started...");
    }

private:
    void publishNews(){
        auto msg = std_msgs::msg::String();
        msg.data = "Hello, I'm " + robot_name_ +", This is a Test Broadcast..";
        publisher_->publish(msg);
    }
    std::string robot_name_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<news_station>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}