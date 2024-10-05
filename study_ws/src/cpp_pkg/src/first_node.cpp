#include "rclcpp/rclcpp.hpp"

class FirstNode : public rclcpp::Node
{  
public:
    FirstNode() : Node("first_node"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello World!");

        timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                         std::bind(&FirstNode::timer_callback, this));
    }
    

private:
    void timer_callback()
    {   
        count_++;
        RCLCPP_INFO(this->get_logger(), "Hello World! %d", count_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int count_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FirstNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

  return 0;
}