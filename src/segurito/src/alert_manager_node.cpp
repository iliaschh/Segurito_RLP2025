#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class AlertManagerNode : public rclcpp::Node
{
    public:
    AlertManagerNode() : Node("alert_manager") 
    {
        m_alert_pub = this->create_publisher<std_msgs::msg::String>("alert", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&AlertManagerNode::publish, this));
        m_motion_detected_sub = this->create_subscription<std_msgs::msg::String>("motion_detected", 10,
                            std::bind(&AlertManagerNode::subCallback, this, std::placeholders::_1));    
        m_person_detected_sub = this->create_subscription<std_msgs::msg::String>("person_detected", 10,
                            std::bind(&AlertManagerNode::subCallback, this, std::placeholders::_1));        
        m_battery_state_sub = this->create_subscription<std_msgs::msg::String>("battery_state", 10,
                            std::bind(&AlertManagerNode::subCallback, this, std::placeholders::_1));   
        RCLCPP_INFO(this->get_logger(), "Alert manager node started");        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Alert manager node publishing");
            m_alert_pub->publish(msg);

        }
        void subCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_alert_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_motion_detected_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_person_detected_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_battery_state_sub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlertManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}