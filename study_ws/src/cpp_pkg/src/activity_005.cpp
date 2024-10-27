#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int8_multi_array.hpp"


class led_states_node : public rclcpp::Node 
{
public:
    led_states_node() : Node("led_states")
    {
        
    }

private:

    rclcpp::Publisher<example_interfaces::msg::Int8MultiArray>::SharedPtr led_states;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<led_states_node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
