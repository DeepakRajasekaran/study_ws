#include <rclcpp/rclcpp.hpp>
#include <cmath>

//IMPORT THE POSE, GEOMETRY_TWIST, 
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class Hunter_Node : public rclcpp::Node 
{
public:
    Hunter_Node() : Node("hunter") 
    {
        RCLCPP_INFO(this->get_logger(), "Hunter_Node has been inititated");


        
    }

private:

    
    
    

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Hunter_Node>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
