#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class NavigationNode : public rclcpp::Node
{
    public:
    NavigationNode() : Node("navigation") 
    {
        m_cmd_vel_pub = this->create_publisher<std_msgs::msg::String>("cmd_vel", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&NavigationNode::publish, this));   
        m_robot_pose_sub = this->create_subscription<std_msgs::msg::String>("robot_pose", 10,
                            std::bind(&NavigationNode::subCallback, this, std::placeholders::_1));        
        RCLCPP_INFO(this->get_logger(), "Navigation node started");        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Navigation node publishing");
            m_cmd_vel_pub->publish(msg);

        }
        void subCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_cmd_vel_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_robot_pose_sub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}