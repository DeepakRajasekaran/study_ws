#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include <irobot_interfaces/msg/turtle_array.hpp>
#include <irobot_interfaces/msg/turtleinfo.hpp>
#include <irobot_interfaces/srv/kill_switch.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class Hunter_Node : public rclcpp::Node 
{
public:
    Hunter_Node() : Node("hunter") 
    {
        RCLCPP_INFO(this->get_logger(), "Hunter_Node has been inititated...");

        //parameters
        this->declare_parameter<int>("kill_closest_turtle_first", false);
        this->kill_closest_turtle_first = this->get_parameter("kill_closest_turtle_first").get_value<int>();

        this->declare_parameter<int>("hunter_freq", 1);
        this->hunter_frequency = this->get_parameter("hunter_frequency").get_value<int>();
        
       // Subscribers and publishers
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&Hunter_Node::pose_subscriber_callback, this));

        target_subscriber = this->create_subscription<irobot_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&Hunter_Node::find_target_pray, this));

        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        // services
        kill_request_ = this->create_client<irobot_interfaces::srv::KillSwitch>("killer");
        
        //timer_callbacks
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Hunter_Node::hunt_pray_callback, this));
        
    }

private:

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Subscription<irobot_interfaces::msg::TurtleArray>::SharedPtr target_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

    rclcpp::Client<irobot_interfaces::srv::KillSwitch>::SharedPtr kill_request_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool kill_closest_turtle_first = false;
    u_int hunter_frequency = 0;

    std::shared_ptr<turtlesim::msg::Pose> self_pose_ = nullptr;

    struct Target 
    {
        bool target_locked = false;
        double target_distance = 0.0;  
        double goal_theta = 0.0;
        std::shared_ptr<irobot_interfaces::msg::Turtleinfo> info;
    };

    void pose_subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg){
        self_pose_ = msg;
    }

    void find_target_pray(/*Input data*/){
        
    }

    void hunt_pray_callback(/* Input data */){
        
    }

    void send_kill_request(/* Input data */){

    }
    
    void target_reset(/*Input data*/){
        // if not target_locked
            //code to reset the target
    }
    
    void kill_request_response_callback(/* Input Data */){

    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Hunter_Node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
