#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp" 

class NumberCounter : public rclcpp::Node {
public:
    NumberCounter() : Node("number_counter"), count_(0) {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "number", 10, std::bind(&NumberCounter::callbackNumber, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number_count", 10);
    }

private:
    void callbackNumber(const std_msgs::msg::Int64::SharedPtr msg) {
        count_ += msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: '%ld', Total: '%ld'", msg->data, count_);
        
        auto new_msg = std_msgs::msg::Int64();
        new_msg.data = count_;
        publisher_->publish(new_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    int64_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounter>());
    rclcpp::shutdown();
    return 0;
}
