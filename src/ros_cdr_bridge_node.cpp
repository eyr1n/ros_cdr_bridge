#include <rclcpp_components/register_node_macro.hpp>

#include "ros_cdr_bridge/ros_cdr_bridge_node.hpp"

namespace ros_cdr_bridge {

RosCdrBridgeNode::RosCdrBridgeNode(const rclcpp::NodeOptions &options)
    : Node{"ros_cdr_bridge", options}, bridge_{this} {
  declare_parameter<int>("port", 9090);
  declare_parameter<int>("qos_depth", 10);

  int port = get_parameter("port").as_int();
  bridge_.start(port);
  thread_ = std::thread{[this]() { bridge_.run(); }};
  RCLCPP_INFO(get_logger(), "websocket server listening on port %d", port);
}

RosCdrBridgeNode::~RosCdrBridgeNode() {
  bridge_.stop();
  thread_.join();
}

} // namespace ros_cdr_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ros_cdr_bridge::RosCdrBridgeNode)
