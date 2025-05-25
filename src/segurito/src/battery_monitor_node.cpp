#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class BatteryMonitorNode : public rclcpp::Node
{
    public:
    BatteryMonitorNode() : Node("battery_monitor") 
    {
        m_battery_monitor_pub = this->create_publisher<std_msgs::msg::String>("battery_state", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&BatteryMonitorNode::publish, this));
        RCLCPP_INFO(this->get_logger(), "Battery monitor node started");                        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Battery monitor node publishing");
            m_battery_monitor_pub->publish(msg);

        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_battery_monitor_pub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}