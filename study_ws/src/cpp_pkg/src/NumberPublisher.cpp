#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp" 
#include <cstdlib>  // For rand()


class NumberPublisher : public rclcpp::Node {
public:
    NumberPublisher() : Node("number_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&NumberPublisher::publishNumber, this));
    }

private:
    void publishNumber() {
        auto message = std_msgs::msg::Int64();
        message.data = rand() % 100;  // Generate a random number between 0 and 99
        RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberPublisher>());
    rclcpp::shutdown();
    return 0;
}
