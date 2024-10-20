#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/srv/set_led.hpp"
#include "irobot_interfaces/msg/led_states.hpp"

class LED_Panel_Node : public rclcpp::Node 
{
public:
    LED_Panel_Node() : Node("LED_Panel_Node") 
    {
        RCLCPP_INFO(this->get_logger(), "LED Panel Node Server has initiated...");
        using std::placeholders::_1;
        using std::placeholders::_2;

        server_ = this->create_service<irobot_interfaces::srv::SetLed>(
            "set_led", std::bind(&LED_Panel_Node::callback_, this, _1, _2));

        led_states = this->create_publisher<irobot_interfaces::msg::LEDStates>("led_states", 10);
    }

private:
    rclcpp::Service<irobot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<irobot_interfaces::msg::LEDStates>::SharedPtr led_states;
    
    void callback_(const std::shared_ptr<irobot_interfaces::srv::SetLed::Request> request,
                   std::shared_ptr<irobot_interfaces::srv::SetLed::Response> response)
    {
        try{
            auto led_state_msg = irobot_interfaces::msg::LEDStates();
            led_state_msg.led_states = request->input_array;
            led_states->publish(led_state_msg);
            std::stringstream ss;
            for (const auto& val : led_state_msg.led_states) ss << val << " ";
            RCLCPP_INFO(this->get_logger(), "LED States Updated: %s", ss.str().c_str());

            response->success = true;
        }
        catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Error occurred: %s", e.what());
            response->success = false;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LED_Panel_Node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
