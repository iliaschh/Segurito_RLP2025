#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class CameraVisionNode : public rclcpp::Node
{
    public:
    CameraVisionNode() : Node("camera_vision") 
    {
        m_image_raw_pub = this->create_publisher<std_msgs::msg::String>("image_raw", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&CameraVisionNode::publish, this));
        m_person_detected_pub = this->create_publisher<std_msgs::msg::String>("person_detected", 10);
        RCLCPP_INFO(this->get_logger(), "Camera vision node started");                        
    }
    private:
        void publish() { 
            auto msg_image_raw = std_msgs::msg::String();
            msg_image_raw.data = std::string("Camera raw node publishing");
            m_image_raw_pub->publish(msg_image_raw);
            auto msg_person_detected = std_msgs::msg::String();
            msg_person_detected.data = std::string("Person detected node publishing"); 
            m_person_detected_pub->publish(msg_person_detected);

        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_image_raw_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_person_detected_pub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraVisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}