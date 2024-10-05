#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <cstdlib>

class Client_SumServer_Node : public rclcpp::Node 
{
public:
    Client_SumServer_Node() : Node("Client_SumServer") 
    {
        srand(time(0));
        RCLCPP_INFO(this->get_logger(), "Client_SumServer is initiated...");
        threads_.push_back(std::thread(
                           std::bind(&Client_SumServer_Node::Client_SumServer_Callback, 
                           this, rand() % 100 + 1, rand() % 100 + 1)));
        threads_.push_back(std::thread(
                           std::bind(&Client_SumServer_Node::Client_SumServer_Callback, 
                           this, rand() % 100 + 1, rand() % 100 + 1)));
    }

    void Client_SumServer_Callback(long a, long b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Server not available...");
        }
        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", a, b, response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service Call Failed...");
        }
    }

private:
    //std::thread thread1_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Client_SumServer_Node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
