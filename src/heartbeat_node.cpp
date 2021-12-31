#include "sh_common/heartbeat_node.hpp"
#include "sh_common/ros_names.hpp"

#define HEARTBEAT_PERIOD_PARAM_NAME sh::names::params::HEARTBEAT_PERIOD_MS
#define HEARTBEAT_TOPIC sh::names::topics::HEARTBEAT

sh::HeartbeatNode::HeartbeatNode(
    const std::string& node_name,
    const std::string& ns,
    const rclcpp::NodeOptions& options)
        : Node(node_name, ns, options) {
    declare_parameter<int>(HEARTBEAT_PERIOD_PARAM_NAME);
    const int period = get_parameter(HEARTBEAT_PERIOD_PARAM_NAME).as_int();
    heartbeat_pub = create_publisher<std_msgs::msg::Header>(
        HEARTBEAT_TOPIC,
        10);

    heartbeat.frame_id = node_name;
    heartbeat_timer = create_wall_timer(
        std::chrono::milliseconds(period),
        [this]() -> void {
            heartbeat.stamp = now();
            heartbeat_pub->publish(heartbeat);
        });
}
