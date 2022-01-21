#include "sh_common/heartbeat_node.hpp"

#include <memory>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const std::string node_name((argc > 1) ? argv[1] : "test_heartbeat_node");
    rclcpp::spin(std::make_shared<sh::HeartbeatNode>(node_name));
    rclcpp::shutdown();
    return 0;
}
