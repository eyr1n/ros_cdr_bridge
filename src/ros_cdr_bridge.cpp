#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros_cdr_bridge/binary_payload.hpp"
#include "ros_cdr_bridge/generic_client.hpp"
#include "ros_cdr_bridge/ros_cdr_bridge.hpp"
#include "ros_cdr_bridge/text_payload.hpp"
#include "ros_cdr_bridge/utility.hpp"

namespace ros_cdr_bridge {

RosCdrBridge::RosCdrBridge(rclcpp::Node *node) : node_{node} {
  callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  server_.clear_access_channels(websocketpp::log::alevel::all);
  server_.clear_error_channels(websocketpp::log::elevel::all);

  server_.init_asio();

  server_.set_open_handler(
      std::bind(&RosCdrBridge::on_open, this, std::placeholders::_1));
  server_.set_close_handler(
      std::bind(&RosCdrBridge::on_close, this, std::placeholders::_1));
  server_.set_message_handler(std::bind(&RosCdrBridge::on_message, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
}

void RosCdrBridge::start(uint16_t port) {
  server_.set_reuse_addr(true);
  server_.listen(port);
  server_.start_accept();
}

void RosCdrBridge::stop() {
  server_.stop_listening();
  server_.stop();
}

void RosCdrBridge::run() { server_.run(); }

void RosCdrBridge::on_open(websocketpp::connection_hdl hdl) {
  sessions_.emplace(hdl, Session{});
  RCLCPP_INFO(node_->get_logger(), "websocket client connected");
}

void RosCdrBridge::on_close(websocketpp::connection_hdl hdl) {
  auto it = sessions_.find(hdl);
  if (it == sessions_.end()) {
    return;
  }
  {
    std::lock_guard lock{mutex_};
    sessions_.erase(it);
    node_->get_node_graph_interface()->notify_graph_change();
  }
  RCLCPP_INFO(node_->get_logger(), "websocket client disconnected");
}

void RosCdrBridge::on_message(
    websocketpp::connection_hdl hdl,
    websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
  auto it = sessions_.find(hdl);
  if (it == sessions_.end()) {
    return;
  }
  Session &session = it->second;

  switch (msg->get_opcode()) {
  case websocketpp::frame::opcode::text:
    std::visit(overloads{
                   [](std::monostate) {},
                   [this, hdl, &session](CreatePublisherRequest &&request) {
                     create_publisher(hdl, session, request.call_id,
                                      request.name, request.type,
                                      request.qos_profile);
                   },
                   [this, hdl, &session](CreateSubscriptionRequest &&request) {
                     create_subscription(hdl, session, request.call_id,
                                         request.name, request.type,
                                         request.qos_profile);
                   },
                   [this, hdl, &session](CreateServiceClientRequest &&request) {
                     create_service_client(hdl, session, request.call_id,
                                           request.name, request.type,
                                           request.qos_profile);
                   },
                   [this, &session](DestroyRequest &&request) {
                     destroy_id(session, request.id);
                   },
               },
               parse_text_payload(msg->get_payload()));
    break;

  case websocketpp::frame::opcode::binary: {
    std::visit(overloads{
                   [](std::monostate) {},
                   [this, &session](PublisherMessage &&message) {
                     auto it = session.entities.find(message.id);
                     if (it == session.entities.end()) {
                       return;
                     }
                     if (auto publisher =
                             std::get_if<rclcpp::GenericPublisher::SharedPtr>(
                                 &it->second)) {
                       publish_message(*publisher, std::move(message.message));
                     }
                   },
                   [this, hdl, &session](ServiceClientRequest &&request) {
                     auto it = session.entities.find(request.id);
                     if (it == session.entities.end()) {
                       return;
                     }
                     if (auto client =
                             std::get_if<std::shared_ptr<GenericClient>>(
                                 &it->second)) {
                       call_service(hdl, *client, request.id, request.call_id,
                                    request.message);
                     }
                   },
               },
               parse_binary_payload(msg->get_payload()));
    break;
  }

  default:
    break;
  }
}

void RosCdrBridge::create_publisher(websocketpp::connection_hdl hdl,
                                    Session &session, uint32_t call_id,
                                    const std::string &name,
                                    const std::string &type,
                                    const rmw_qos_profile_t &qos_profile) {
  uint32_t id = session.next_id++;
  rclcpp::GenericPublisher::SharedPtr publisher;
  {
    std::lock_guard lock{mutex_};
    rclcpp::PublisherOptions options;
    options.callback_group = callback_group_;
    publisher = node_->create_generic_publisher(
        name, type,
        rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(qos_profile),
                    qos_profile},
        options);
  }

  session.entities.emplace(id, publisher);
  nlohmann::json response;
  response["id"] = id;
  response["callId"] = call_id;
  send_text_payload(hdl, response.dump());
  RCLCPP_INFO(node_->get_logger(), "create_publisher id=%u name=%s type=%s", id,
              name.c_str(), type.c_str());
}

void RosCdrBridge::create_subscription(websocketpp::connection_hdl hdl,
                                       Session &session, uint32_t call_id,
                                       const std::string &name,
                                       const std::string &type,
                                       const rmw_qos_profile_t &qos_profile) {
  uint32_t id = session.next_id++;
  rclcpp::GenericSubscription::SharedPtr subscription;
  {
    std::lock_guard lock{mutex_};
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription = node_->create_generic_subscription(
        name, type,
        rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(qos_profile),
                    qos_profile},
        [this, hdl, id](std::shared_ptr<rclcpp::SerializedMessage> message) {
          const auto &rcl_msg = message->get_rcl_serialized_message();
          std::vector<uint8_t> payload(1 + sizeof(id) + rcl_msg.buffer_length);
          payload[0] = OP_TOPIC;
          std::memcpy(payload.data() + 1, &id, sizeof(id));
          std::memcpy(payload.data() + 1 + sizeof(id), rcl_msg.buffer,
                      rcl_msg.buffer_length);
          send_binary_payload(hdl, std::move(payload));
        },
        options);
  }

  session.entities.emplace(id, subscription);
  nlohmann::json response;
  response["id"] = id;
  response["callId"] = call_id;
  send_text_payload(hdl, response.dump());
  RCLCPP_INFO(node_->get_logger(), "create_subscription id=%u name=%s type=%s",
              id, name.c_str(), type.c_str());
}

void RosCdrBridge::create_service_client(websocketpp::connection_hdl hdl,
                                         Session &session, uint32_t call_id,
                                         const std::string &name,
                                         const std::string &type,
                                         const rmw_qos_profile_t &qos_profile) {
  uint32_t id = session.next_id++;
  std::shared_ptr<GenericClient> client;
  {
    std::lock_guard lock{mutex_};
    auto options = rcl_client_get_default_options();
    options.qos = qos_profile;
    client = std::make_shared<GenericClient>(
        node_->get_node_base_interface().get(),
        node_->get_node_graph_interface(), name, type, options);
    node_->get_node_services_interface()->add_client(client, callback_group_);
  }

  session.entities.emplace(id, client);
  nlohmann::json response;
  response["id"] = id;
  response["callId"] = call_id;
  send_text_payload(hdl, response.dump());
  RCLCPP_INFO(node_->get_logger(),
              "create_service_client id=%u name=%s type=%s", id, name.c_str(),
              type.c_str());
}

void RosCdrBridge::destroy_id(Session &session, uint32_t id) {
  auto it = session.entities.find(id);
  if (it == session.entities.end()) {
    return;
  }
  {
    std::lock_guard lock{mutex_};
    session.entities.erase(it);
    node_->get_node_graph_interface()->notify_graph_change();
  }
  RCLCPP_INFO(node_->get_logger(), "destroy id=%u", id);
}

void RosCdrBridge::publish_message(
    const rclcpp::GenericPublisher::SharedPtr pub,
    rclcpp::SerializedMessage &&message) {
  std::shared_lock lock{mutex_};
  pub->publish(std::move(message));
}

void RosCdrBridge::call_service(websocketpp::connection_hdl hdl,
                                const std::shared_ptr<GenericClient> client,
                                uint32_t id, uint32_t call_id,
                                const rclcpp::SerializedMessage &message) {
  std::shared_lock lock{mutex_};
  client->async_send_request(message, [this, hdl, id, call_id](
                                          const uint8_t *data, size_t size) {
    std::vector<uint8_t> payload(1 + sizeof(id) + sizeof(call_id) + size);
    payload[0] = OP_SERVICE_RESPONSE;
    std::memcpy(payload.data() + 1, &id, sizeof(id));
    std::memcpy(payload.data() + 1 + sizeof(id), &call_id, sizeof(call_id));
    std::memcpy(payload.data() + 1 + sizeof(id) + sizeof(call_id), data, size);
    send_binary_payload(hdl, std::move(payload));
  });
}

void RosCdrBridge::send_text_payload(websocketpp::connection_hdl hdl,
                                     std::string &&payload) {
  server_.get_io_service().post([this, hdl, payload = std::move(payload)]() {
    websocketpp::lib::error_code ec;
    server_.send(hdl, payload, websocketpp::frame::opcode::text, ec);
  });
}

void RosCdrBridge::send_binary_payload(websocketpp::connection_hdl hdl,
                                       std::vector<uint8_t> &&payload) {
  server_.get_io_service().post([this, hdl, payload = std::move(payload)]() {
    websocketpp::lib::error_code ec;
    server_.send(hdl, payload.data(), payload.size(),
                 websocketpp::frame::opcode::binary, ec);
  });
}

} // namespace ros_cdr_bridge
