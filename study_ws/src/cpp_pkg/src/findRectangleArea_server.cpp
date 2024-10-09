#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/srv/rectangle_parameters.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class findRectangleArea : public rclcpp::Node 
{
public:
    findRectangleArea() : Node("serverNode") 
    {
        RCLCPP_INFO(this->get_logger(), "computeRectangleArea Server has been initiated...");
        
        // Ensure the service name matches the client-side name
        server_ = this->create_service<irobot_interfaces::srv::RectangleParameters>(
            "clientRequest", std::bind(&findRectangleArea::compute_callback, this, _1, _2));
    }

private:
    void compute_callback(const irobot_interfaces::srv::RectangleParameters::Request::SharedPtr request,
                          const irobot_interfaces::srv::RectangleParameters::Response::SharedPtr response)
    {
        response->area = request->length * request->width;
        RCLCPP_INFO(this->get_logger(), "Received request: length=%.2f, width=%.2f, computed area=%.2f",
                    request->length, request->width, response->area);
    }

    rclcpp::Service<irobot_interfaces::srv::RectangleParameters>::SharedPtr server_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<findRectangleArea>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
