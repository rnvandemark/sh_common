#include "sh_common/heartbeat_node.hpp"

#define HEARTPEAD_PERIOD_PARAM_NAME "heartbeat_period"

sh::HeartbeatNode::HeartbeatNode(
    const std::string& node_name,
    const rclcpp::NodeOptions& options)
        : Node(node_name, options) {
    declare_parameter<int>(HEARTPEAD_PERIOD_PARAM_NAME);
    const int period = get_parameter(HEARTPEAD_PERIOD_PARAM_NAME).as_int();
    heartbeat_pub = create_publisher<std_msgs::msg::Header>(
        "heartbeat",
        10);

    heartbeat.frame_id = node_name;
    heartbeat_timer = create_wall_timer(
        std::chrono::milliseconds(period),
        [this]() -> void {
            heartbeat.stamp = now();
            heartbeat_pub->publish(heartbeat);
        });
}