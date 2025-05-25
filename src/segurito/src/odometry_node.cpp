#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class OdometryNode : public rclcpp::Node
{
    public:
    OdometryNode() : Node("odom") 
    {
        m_odom_pub = this->create_publisher<std_msgs::msg::String>("odom", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&OdometryNode::publish, this));
        RCLCPP_INFO(this->get_logger(), "Odometry node started");         
        m_wheel_ticks_sub = this->create_subscription<std_msgs::msg::String>("wheel_ticks", 10,
                            std::bind(&OdometryNode::wheelTicksCallback, this, std::placeholders::_1));               
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Odometry node publishing");
            m_odom_pub->publish(msg);

        }
        void wheelTicksCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_odom_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_wheel_ticks_sub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}