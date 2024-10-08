#include "rclcpp/rclcpp.hpp"
#include "irobot_interfaces/srv/rectangle_parameters.hpp"
#include <cstdlib>

using ServiceResponseFuture = rclcpp::Client<irobot_interfaces::srv::RectangleParameters>::SharedFuture;

class clientNode : public rclcpp::Node 
{
public:
    clientNode() : Node("client_node") 
    {
        RCLCPP_INFO(this->get_logger(), "ClientNode has been initiated...");

        // Send a client request when the node is initiated
        client_request();
    }

private:
    void client_request()
    {
        srand(time(0));

        // Create a client for the RectangleParameters service
        auto client = this->create_client<irobot_interfaces::srv::RectangleParameters>("clientRequest");

        // Wait for the service to be available
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Server not available, waiting again...");
        }

        // Prepare the request with random length and width values
        auto request = std::make_shared<irobot_interfaces::srv::RectangleParameters::Request>();
        request->length = rand() % 100 + 1;
        request->width = rand() % 100 + 1;
        RCLCPP_INFO(this->get_logger(), "Sending request: length = %.2f, width = %.2f", request->length, request->width);

        // Send the request asynchronously and bind the response callback
        auto future = client->async_send_request(request,
            std::bind(&clientNode::response_callback, this, std::placeholders::_1)
        );
    }

    // Response callback to handle the service response
    void response_callback(ServiceResponseFuture future)
    {
        try {
            // Get the response from the future
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Received response: Area = %.2f", response->area);
        } catch (const std::exception &e) {
            // Handle any errors
            RCLCPP_ERROR(this->get_logger(), "Error occurred: %s", e.what());
        }
    }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<clientNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
