#include "sh_common/heartbeat_node.hpp"
#include "sh_common/ros_names.hpp"

#include <memory>

namespace sh {
class HeartbeatListenerNode : public sh::HeartbeatNode
{
protected:
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_sub;

public:
    HeartbeatListenerNode(
        const std::string& node_name,
        const std::string& other_node_name) :
            sh::HeartbeatNode(node_name)
    {
        heartbeat_sub = create_subscription<std_msgs::msg::Header>(
            sh::names::topics::HEARTBEAT_PREFIX + "/" + other_node_name,
            10,
            [this](std_msgs::msg::Header::UniquePtr msg) {
                RCLCPP_INFO(
                    get_logger(),
                    "Received heartbeat: %s => %u:%u",
                    msg->frame_id.c_str(),
                    msg->stamp.sec,
                    msg->stamp.nanosec);
            }
        );
    }
};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const std::string other_node_name((argc > 1) ? argv[1] : "test_heartbeat_node");
    const std::string node_name((argc > 2) ? argv[2] : "heartbeat_listener_node");
    rclcpp::spin(std::make_shared<sh::HeartbeatListenerNode>(node_name, other_node_name));
    rclcpp::shutdown();
    return 0;
}
