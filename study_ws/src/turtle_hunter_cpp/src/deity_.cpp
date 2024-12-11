#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "irobot_interfaces/msg/turtleinfo.hpp"
#include "irobot_interfaces/msg/turtle_array.hpp"
#include "irobot_interfaces/srv/kill_switch.hpp"

#include <random>
#include <cmath>
#include <functional>

class DeityNode : public rclcpp::Node
{
public:
    DeityNode() : Node("spawner_"), turtle_count(0)
    {
        RCLCPP_INFO(this->get_logger(), "Deity Node has been initiated...");

        this->declare_parameter("spawn_freq", 1.0);
        spawn_period_ = 1.0 / this->get_parameter("spawn_freq").as_double();

        aliveTurtles_publisher_ = this->create_publisher<irobot_interfaces::msg::TurtleArray>("alive_turtles", 10);

        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        killer_ = this->create_client<turtlesim::srv::Kill>("kill");
        kill_command_ = this->create_service<irobot_interfaces::srv::KillSwitch>("killer", std::bind(&DeityNode::kill_command_callback, this, std::placeholders::_1, std::placeholders::_2));

        spawn_timer_ = this->create_wall_timer(std::chrono::duration<double>(spawn_period_), std::bind(&DeityNode::timer_callback_, this));
    }

private:
    void timer_callback_()
    {
        if (!spawner_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Spawn service is unavailable. Retrying...");
            return;
        }
        spawn_turtle();
    }

    void spawn_turtle()
    {
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = static_cast<float>(rand() % 10 + 1);
        request->y = static_cast<float>(rand() % 10 + 1);
        request->theta = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * M_PI - M_PI;
        turtle_count++;
        request->name = "Pray_" + std::to_string(turtle_count);

        auto future = spawner_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            log_spawned_turtle(request, future.share());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle");
        }
    }

    void log_spawned_turtle(std::shared_ptr<turtlesim::srv::Spawn::Request> request, rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Turtle %s born at x: %f, y: %f", response->name.c_str(), request->x, request->y);
            irobot_interfaces::msg::Turtleinfo born_turtle;
            born_turtle.x = request->x;
            born_turtle.y = request->y;
            born_turtle.theta = request->theta;
            born_turtle.name = response->name;
            alive_turtles.push_back(born_turtle);
            publish_alive_turtles();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Turtle died during birth: %s", e.what());
        }
    }

    void kill_command_callback(const std::shared_ptr<irobot_interfaces::srv::KillSwitch::Request> request, std::shared_ptr<irobot_interfaces::srv::KillSwitch::Response> response)
    {
        kill_turtle(request->name);
        response->killed = true;
    }

    void kill_turtle(const std::string &turtle_ID)
    {
        if (!killer_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Kill service is unavailable. Retrying...");
            return;
        }

        auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        kill_request->name = turtle_ID;
        auto future = killer_->async_send_request(kill_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
        {
            log_killed_turtle(turtle_ID, future.share());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to kill turtle %s", turtle_ID.c_str());
        }
    }

    void log_killed_turtle(const std::string &turtle_ID, rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
    {
        try
        {
            future.get();
            auto it = std::remove_if(alive_turtles.begin(), alive_turtles.end(), [&turtle_ID](const irobot_interfaces::msg::Turtleinfo &turtle) {
                return turtle.name == turtle_ID;
            });
            alive_turtles.erase(it, alive_turtles.end());
            RCLCPP_WARN(this->get_logger(), "%s is killed by hunter.", turtle_ID.c_str());
            publish_alive_turtles();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "%s escaped from hunter due to: %s", turtle_ID.c_str(), e.what());
        }
    }

    void publish_alive_turtles()
    {
        auto msg = irobot_interfaces::msg::TurtleArray();
        msg.turtles = alive_turtles;
        aliveTurtles_publisher_->publish(msg);
    }

    rclcpp::Publisher<irobot_interfaces::msg::TurtleArray>::SharedPtr aliveTurtles_publisher_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr killer_;
    rclcpp::Service<irobot_interfaces::srv::KillSwitch>::SharedPtr kill_command_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
    double spawn_period_;
    int turtle_count;
    std::vector<irobot_interfaces::msg::Turtleinfo> alive_turtles;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
