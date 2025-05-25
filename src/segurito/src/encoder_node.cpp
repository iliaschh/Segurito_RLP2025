#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class EncoderNode : public rclcpp::Node
{
    public:
    EncoderNode() : Node("encoder") 
    {
        m_wheel_ticks_pub = this->create_publisher<std_msgs::msg::String>("wheel_ticks", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&EncoderNode::publish, this));
        RCLCPP_INFO(this->get_logger(), "Encoder node started");                        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Encoder node publishing");
            m_wheel_ticks_pub->publish(msg);

        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_wheel_ticks_pub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}