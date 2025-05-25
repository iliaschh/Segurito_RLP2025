#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class SLAMNode : public rclcpp::Node
{
    public:
    SLAMNode() : Node("slam") 
    {
        m_map_pub = this->create_publisher<std_msgs::msg::String>("map", 10);
        m_robot_pose_pub = this->create_publisher<std_msgs::msg::String>("robot_pose", 10);
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), 
                                    std::bind(&SLAMNode::publish, this));
        m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
                            std::bind(&SLAMNode::laserCallback, this, std::placeholders::_1));    
        m_odom_sub = this->create_subscription<std_msgs::msg::String>("odom", 10,
                            std::bind(&SLAMNode::subCallback, this, std::placeholders::_1));        
        RCLCPP_INFO(this->get_logger(), "Odometry node started");        
    }
    private:
        void publish() { 
            auto msg = std_msgs::msg::String();
            msg.data = std::string("SLAM node publishing");
            m_map_pub->publish(msg);
            m_robot_pose_pub->publish(msg);

        }
        void subCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            return;
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_map_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_robot_pose_pub;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_odom_sub;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SLAMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}