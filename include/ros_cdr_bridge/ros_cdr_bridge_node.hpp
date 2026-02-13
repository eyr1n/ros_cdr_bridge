#pragma once

#include <rclcpp/rclcpp.hpp>

#include "ros_cdr_bridge/ros_cdr_bridge.hpp"

namespace ros_cdr_bridge {

class RosCdrBridgeNode : public rclcpp::Node {
public:
  explicit RosCdrBridgeNode(const rclcpp::NodeOptions &options);
  ~RosCdrBridgeNode() override;

private:
  RosCdrBridge bridge_;
};

} // namespace ros_cdr_bridge
