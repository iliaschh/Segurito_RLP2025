#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class MapServerNode : public rclcpp::Node
{
    public:
    MapServerNode() : Node("map_server") 
    { 
        m_map_sub = this->create_subscription<std_msgs::msg::String>("map", 10,
                            std::bind(&MapServerNode::subCallback, this, std::placeholders::_1));        
        RCLCPP_INFO(this->get_logger(), "Map server node started");        
    }
    private:
        void subCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            return;
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_map_sub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}