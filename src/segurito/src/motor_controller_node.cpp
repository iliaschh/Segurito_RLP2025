#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class MotorControllerNode : public rclcpp::Node
{
    public:
    MotorControllerNode() : Node("motor_controller") 
    {
        m_cmd_vel_sub = this->create_subscription<std_msgs::msg::String>("cmd_vel", 10,
                            std::bind(&MotorControllerNode::subCallback, this, std::placeholders::_1));        
        RCLCPP_INFO(this->get_logger(), "Motor controller node started");        
    }
    private:
        void subCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_cmd_vel_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}