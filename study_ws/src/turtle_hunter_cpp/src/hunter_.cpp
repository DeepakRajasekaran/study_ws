#include "rclcpp/rclcpp.hpp"
#include "math.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "irobot_interfaces/msg/turtle_array.hpp"
#include "irobot_interfaces/srv/kill_switch.hpp"
#include <thread>
#include <mutex>
#include <vector>
#include <limits>

class HunterNode : public rclcpp::Node
{
public:
    HunterNode() : Node("hunter")
    {
        RCLCPP_INFO(this->get_logger(), "Hunter Node has been initiated...");

        pose_          = nullptr;
        target_locked_ = false;

        // Declare and get parameters
        this->declare_parameter<bool>("kill_closest_turtle_first", false);
        this->declare_parameter<int>("hunter_freq", 10);
        this->declare_parameter<double>("kp_linear", 0.8);
        this->declare_parameter<double>("kp_angular", 6.0);
        this->declare_parameter<bool>("test_mode", false);

        kill_closest_turtle_first_ = this->get_parameter("kill_closest_turtle_first").as_bool();
        hunter_freq_               = 1.0 / this->get_parameter("hunter_freq").as_int();
        kp_linear_                 = this->get_parameter("kp_linear").as_double();
        kp_angular_                = this->get_parameter("kp_angular").as_double();
        test_mode_                 = this->get_parameter("test_mode").as_bool();

        // Subscribers, Publisher, and Client
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10,
            std::bind(&HunterNode::poseSubscriberCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        target_subscriber_ = this->create_subscription<irobot_interfaces::msg::TurtleArray>("alive_turtles", 1,
            std::bind(&HunterNode::findTargetPray, this, std::placeholders::_1));

        test_vertex_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>("test_vertex", 10,
            std::bind(&HunterNode::testVertexCallback, this, std::placeholders::_1));

        kill_request_client_ = this->create_client<irobot_interfaces::srv::KillSwitch>("killer");

        timer_ = this->create_wall_timer(std::chrono::duration<double>(hunter_freq_),
            std::bind(&HunterNode::timerCallback, this));
    }

    ~HunterNode()
    {
        for (auto &thread : threads_)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
    }

private:
    void poseSubscriberCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = msg;
    }

    void testVertexCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        test_vertex_ = msg;
    }

    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        double dist_x = x2 - x1;
        double dist_y = y2 - y1;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    std::shared_ptr<irobot_interfaces::msg::Turtleinfo> findClosestTurtle(
        const std::vector<irobot_interfaces::msg::Turtleinfo> &turtles,
        const turtlesim::msg::Pose::SharedPtr &pose)
    {
        std::shared_ptr<irobot_interfaces::msg::Turtleinfo> closest_turtle = nullptr;
        double closest_distance = std::numeric_limits<double>::max();

        for (const auto &turtle : turtles)
        {
            double distance = calculateDistance(pose->x, pose->y, turtle.x, turtle.y);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_turtle = std::make_shared<irobot_interfaces::msg::Turtleinfo>(turtle);
            }
        }

        return closest_turtle;
    }

    void findTargetPray(const irobot_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (kill_closest_turtle_first_)
        {
            if (!target_locked_)
            {
                target_ = findClosestTurtle(msg->turtles, pose_);
                target_distance_ = calculateDistance(pose_->x, pose_->y, target_->x, target_->y);
            }
            else
            {
                target_distance_ = calculateDistance(pose_->x, pose_->y, target_->x, target_->y);
            }

            goal_theta_ = std::atan2(target_->y - pose_->y, target_->x - pose_->x);
            target_locked_ = true;

            RCLCPP_INFO(this->get_logger(), "\033[92mtarget_locked..\033[0m");
        }
        else
        {
            target_ = std::make_shared<irobot_interfaces::msg::Turtleinfo>(msg->turtles[0]);
            target_distance_ = calculateDistance(pose_->x, pose_->y, target_->x, target_->y);
            goal_theta_ = std::atan2(target_->y - pose_->y, target_->x - pose_->x);
        }
    }

    void timerCallback()
    {
        test_mode_ ? driveTestCallback() : huntPrayCallback();
    }

    void huntPrayCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pose_ || !target_){
            RCLCPP_ERROR(this->get_logger(), "Error: Pose or target is not available.");
            return;
        }
        geometry_msgs::msg::Twist msg;

        if (target_distance_ > 0.5)
        {
            msg.linear.x  = kp_linear_ * target_distance_;
            double diff = std::fmod(goal_theta_ - pose_->theta + M_PI, 2 * M_PI) - M_PI;
            msg.angular.z = kp_angular_ * diff;
        }
        else
        {
            msg.linear.x  = 0.0;
            msg.angular.z = 0.0;
            threads_.emplace_back(&HunterNode::sendKillRequest, this, target_->name);
        }

        cmd_vel_publisher_->publish(msg);
    }

    void driveTestCallback()
    {
        RCLCPP_WARN(this->get_logger(), "Test mode is enabled. Driving to test vertex...");
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pose_ || !test_vertex_){
            RCLCPP_ERROR(this->get_logger(), "Error: Pose or test vertex is not available.");
            return;
        }
        geometry_msgs::msg::Twist msg;

        double test_x = test_vertex_->x;
        double test_y = test_vertex_->y;

        double dist_x = test_x - pose_->x;
        double dist_y = test_y - pose_->y;
        double distance = calculateDistance(pose_->x, pose_->y, test_x, test_y);

        if (distance > 0.5)
        {
            msg.linear.x  = kp_linear_ * distance;
            double goal_theta = std::atan2(dist_y, dist_x);
            double diff = std::fmod(goal_theta - pose_->theta + M_PI, 2 * M_PI) - M_PI;
            msg.angular.z = kp_angular_ * diff;
        }
        else
        {
            msg.linear.x  = 0.0;
            msg.angular.z = 0.0;
        }

        cmd_vel_publisher_->publish(msg);
    }

    void sendKillRequest(const std::string &turtle_name)
    {
        if (!kill_request_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the server.");
            return;
        }

        auto request = std::make_shared<irobot_interfaces::srv::KillSwitch::Request>();
        request->name = turtle_name;

        auto future = kill_request_client_->async_send_request(request);

        future.wait(); // Ensure the call completes before proceeding

        try
        {
            auto response = future.get();
            if (!response->killed)
            {
                RCLCPP_INFO(this->get_logger(), "Pray escaped due to unknown reasons.");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Error occurred during kill request: %s", e.what());
        }

        std::lock_guard<std::mutex> lock(mutex_);
        // Reset target-related data
        if (target_locked_)
        {
            target_.reset();
            target_distance_ = 0.0;
            target_locked_ = false;
            goal_theta_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "\033[92mtarget_resetted..\033[0m");
        }
    }

    // Node variables
    turtlesim::msg::Pose::SharedPtr pose_;
    std::shared_ptr<irobot_interfaces::msg::Turtleinfo> target_;
    std::shared_ptr<geometry_msgs::msg::Pose2D> test_vertex_;

    double target_distance_;
    double goal_theta_;
    bool target_locked_;
    bool kill_closest_turtle_first_;
    double kp_linear_;
    double kp_angular_;
    bool test_mode_;
    double hunter_freq_;

    // Mutex for thread safety
    std::mutex mutex_;

    // Subscribers, Publisher, and Client
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<irobot_interfaces::msg::TurtleArray>::SharedPtr target_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr test_vertex_subscriber_;
    rclcpp::Client<irobot_interfaces::srv::KillSwitch>::SharedPtr kill_request_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Threads for handling asynchronous service calls
    std::vector<std::thread> threads_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HunterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
