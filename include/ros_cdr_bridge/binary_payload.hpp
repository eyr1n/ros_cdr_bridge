#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/serialized_message.hpp>

namespace ros_cdr_bridge {

inline constexpr uint8_t OP_TOPIC = 1;
inline constexpr uint8_t OP_SERVICE_REQUEST = 2;
inline constexpr uint8_t OP_SERVICE_RESPONSE = 3;

struct PublisherMessage {
  uint32_t id;
  rclcpp::SerializedMessage message;
};

struct ServiceClientRequest {
  uint32_t id;
  uint32_t call_id;
  rclcpp::SerializedMessage message;
};

inline void to_serialized_message(rclcpp::SerializedMessage &msg,
                                  const char *data, size_t size) {
  if (msg.capacity() < size) {
    msg.reserve(size);
  }
  auto &rcl_msg = msg.get_rcl_serialized_message();
  std::memcpy(rcl_msg.buffer, data, size);
  rcl_msg.buffer_length = size;
}

inline std::variant<std::monostate, PublisherMessage, ServiceClientRequest>
parse_binary_payload(const std::string &payload) {
  if (payload.size() < 1) {
    return std::monostate{};
  }
  uint8_t op = payload[0];

  if (op == OP_TOPIC) {
    if (payload.size() < 5) {
      return std::monostate{};
    }
    PublisherMessage message;
    std::memcpy(&message.id, payload.data() + 1, sizeof(message.id));
    to_serialized_message(message.message, payload.data() + 5,
                          payload.size() - 5);
    return message;
  }

  if (op == OP_SERVICE_REQUEST) {
    if (payload.size() < 9) {
      return std::monostate{};
    }
    ServiceClientRequest request;
    std::memcpy(&request.id, payload.data() + 1, sizeof(request.id));
    std::memcpy(&request.call_id, payload.data() + 5, sizeof(request.call_id));
    to_serialized_message(request.message, payload.data() + 9,
                          payload.size() - 9);
    return request;
  }

  return std::monostate{};
}

} // namespace ros_cdr_bridge
