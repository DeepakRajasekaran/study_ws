#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/set_bool.hpp" 

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounter : public rclcpp::Node {
public:
    NumberCounter() : Node("number_counter"), count_(0), history_(0) {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "number", 10, std::bind(&NumberCounter::callbackNumber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);

        counter_reset_service_ = this->create_service<std_srvs::srv::SetBool>(
            "reset_counter", std::bind(&NumberCounter::reset_callback, this, _1, _2));
    }

private:
    void callbackNumber(const std_msgs::msg::Int64::SharedPtr msg) {
        if(msg->data != history_){
            count_ += 1;
            auto new_msg = std_msgs::msg::Int64();
            new_msg.data = count_;
            publisher_->publish(new_msg);
            history_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received: '%ld', Instance: '%ld'", msg->data, count_);
        }
    }

    void reset_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {   
        if(request->data){
            count_ = 0;
            response->success = true;
            response->message = "counter_resetted...";
            RCLCPP_INFO(this->get_logger(),"%s", response->message.c_str());
        }
        else{
            response->success = false;
            response->message = "counter_not_resetted...";
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr counter_reset_service_;
    int64_t count_;
    int64_t history_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounter>());
    rclcpp::shutdown();
    return 0;
}
