#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/srv/set_led.hpp"
#include "irobot_interfaces/msg/led_states.hpp"
#include <cstdlib>
#include <array>

using ServiceResponseFuture = rclcpp::Client<irobot_interfaces::srv::SetLed>::SharedFuture;

class BatteryNode : public rclcpp::Node 
{
public:
    BatteryNode() : Node("battery_node")
    {
        srand(time(0));
        RCLCPP_INFO(this->get_logger(), "Battery Node has initiated...");
        client_ = this->create_client<irobot_interfaces::srv::SetLed>("set_led"); 
        loop_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BatteryNode::timer_callback, this));
    }

    ~BatteryNode() {
        for (auto &thread : threads_) {
            if (thread.joinable()) {
                thread.join();  // Ensure all threads are joined before destruction
            }
        }
    }


private:
    rclcpp::Client<irobot_interfaces::srv::SetLed>::SharedPtr client_;
    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr loop_;
    std::string green = "\033[92m";
    std::string endc = "\033[0m";
    short charge = 0;

    void timer_callback(){
        if (!client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "Service 'set_led' not available yet. Retrying...");
        }
        else {threads_.emplace_back(std::thread(&BatteryNode::handler, this));}
    }

    void handler(){response(client_request());}

    ServiceResponseFuture client_request(){

        auto request = std::make_shared<irobot_interfaces::srv::SetLed::Request>();
        charge = BatteryLevel();
        request->input_array = process_led_states(request->input_array, charge); // Fixed for std::array<bool, 4>
        std::stringstream ss;
        for (const auto& val : request->input_array) ss << val << " ";
        RCLCPP_INFO(this->get_logger(), "Determined LED States: %s", ss.str().c_str());
        try{
            auto future = client_->async_send_request(request).share(); // Updated: Use .share()
            return future;
        }
        catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Error occurred during service call: %s", e.what());
            return ServiceResponseFuture(); // Fixed: Return default-constructed future
        }
    }

    void response(ServiceResponseFuture future){
        if (!future.valid()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid future object.");
            return;
        }
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "%sLED Array Updated | Battery Percent: %d%%%s", green.c_str(), charge, endc.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "LED States Update Failed");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error occurred: %s", e.what());
        }
    }

    short int BatteryLevel(){return rand() % 100 + 1;}

    std::array<bool, 4> process_led_states(std::array<bool, 4>& array, short int charge){ // Fixed for std::array
        if (charge < 25) {
            std::fill(array.begin(), array.end(), false);
        } 
        else if (charge < 50) {
            array[0] = true;
            std::fill(array.begin() + 1, array.end(), false);
        } 
        else if (charge < 75) {
            array[0] = true;
            array[1] = true;
            std::fill(array.begin() + 2, array.end(), false);
        } 
        else if (charge > 75 && charge < 95) {
            array[0] = true;
            array[1] = true;
            array[2] = true;
            std::fill(array.begin() + 3, array.end(), false);
        } 
        else if (charge >= 95) {
            std::fill(array.begin(), array.end(), true);
        }

        return array;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
