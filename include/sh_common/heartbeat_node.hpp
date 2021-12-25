#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace sh {
class HeartbeatNode : public rclcpp::Node {
protected:
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    std_msgs::msg::Header heartbeat;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_pub;

public:
    HeartbeatNode(const std::string& node_name,
                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};
}
