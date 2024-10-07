#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class sumServerNode : public rclcpp::Node 
{
public:
    sumServerNode() : Node("sum_server") 
    {   
        RCLCPP_INFO(this->get_logger(), "sumServer is initiated...");
        server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints", std::bind(&sumServerNode::sumServer_callback, this, _1, _2));
    }

private:

    void sumServer_callback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                            const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum=request->a + request->b;
        RCLCPP_INFO(this->get_logger(),"%ld + %ld = %ld", request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sumServerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}