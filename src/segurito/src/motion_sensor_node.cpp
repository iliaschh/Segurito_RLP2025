#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class MotionSensorNode : public rclcpp::Node
{
    public:
    MotionSensorNode() : Node("motion_sensor") 
    {
        m_motion_detected_pub = this->create_publisher<std_msgs::msg::String>("motion_detected", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&MotionSensorNode::publish, this));
        m_imu_pub = this->create_publisher<std_msgs::msg::String>("imu", 10);
        RCLCPP_INFO(this->get_logger(), "Motion sensor node started");                        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("Motion sensor node publishing");
            m_motion_detected_pub->publish(msg);
            m_imu_pub->publish(msg);

        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_motion_detected_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_imu_pub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionSensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}