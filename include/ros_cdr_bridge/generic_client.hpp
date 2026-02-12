#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/client.hpp>

#include "ros_cdr_bridge/message_serialization.hpp"

namespace ros_cdr_bridge {

class GenericClient : public rclcpp::ClientBase {
public:
  explicit GenericClient(
      rclcpp::node_interfaces::NodeBaseInterface *node_base,
      rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
      const std::string &name, const std::string &type,
      rcl_client_options_t &options);
  std::shared_ptr<void> create_response() override;
  std::shared_ptr<rmw_request_id_t> create_request_header() override;
  void handle_response(std::shared_ptr<rmw_request_id_t> request_header,
                       std::shared_ptr<void> response) override;
  void
  async_send_request(const rclcpp::SerializedMessage &request,
                     std::function<void(const uint8_t *, size_t)> &&callback);

private:
  std::mutex mutex_;
  std::unordered_map<int64_t, std::function<void(const uint8_t *, size_t)>>
      callbacks_;
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib_;
  const rosidl_service_type_support_t *typesupport_handle_;

  MessageAllocator request_allocator_;
  MessageAllocator response_allocator_;
  MessageDeserializer request_deserializer_;
  MessageSerializer response_serializer_;
};

} // namespace ros_cdr_bridge
