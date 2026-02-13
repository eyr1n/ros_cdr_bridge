#pragma once

#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rmw/types.h>

#include "ros_cdr_bridge/generic_client.hpp"

namespace ros_cdr_bridge {

class RosCdrBridge {
public:
  explicit RosCdrBridge(rclcpp::Node *node);

  void start(uint16_t port);
  void stop();

private:
  struct Session {
    std::unordered_map<uint32_t,
                       std::variant<rclcpp::GenericPublisher::SharedPtr,
                                    rclcpp::GenericSubscription::SharedPtr,
                                    std::shared_ptr<GenericClient>>>
        entities;
    uint32_t next_id = 0;
  };

  void on_open(websocketpp::connection_hdl hdl);
  void on_close(websocketpp::connection_hdl hdl);
  void
  on_message(websocketpp::connection_hdl hdl,
             websocketpp::server<websocketpp::config::asio>::message_ptr msg);

  void create_publisher(websocketpp::connection_hdl hdl, Session &session,
                        uint32_t call_id, const std::string &name,
                        const std::string &type,
                        const rmw_qos_profile_t &qos_profile);
  void create_subscription(websocketpp::connection_hdl hdl, Session &session,
                           uint32_t call_id, const std::string &name,
                           const std::string &type,
                           const rmw_qos_profile_t &qos_profile);
  void create_service_client(websocketpp::connection_hdl hdl, Session &session,
                             uint32_t call_id, const std::string &name,
                             const std::string &type,
                             const rmw_qos_profile_t &qos_profile);
  void destroy_id(Session &session, uint32_t id);

  void publish_message(const rclcpp::GenericPublisher::SharedPtr pub,
                       rclcpp::SerializedMessage &&message);
  void call_service(websocketpp::connection_hdl hdl,
                    const std::shared_ptr<GenericClient> client,
                    uint32_t call_id, const rclcpp::SerializedMessage &message);

  void send_text_payload(websocketpp::connection_hdl hdl,
                         std::string &&payload);
  void send_binary_payload(websocketpp::connection_hdl hdl,
                           std::vector<uint8_t> &&payload);

  std::shared_mutex node_mutex_;
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  websocketpp::server<websocketpp::config::asio> server_;
  std::thread server_thread_;
  std::map<websocketpp::connection_hdl, Session,
           std::owner_less<websocketpp::connection_hdl>>
      sessions_;
};

} // namespace ros_cdr_bridge
