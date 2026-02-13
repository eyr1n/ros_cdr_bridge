#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <variant>

#include <nlohmann/json.hpp>

namespace ros_cdr_bridge {

inline constexpr std::string_view OP_CREATE_SUBSCRIPTION =
    "create_subscription";
inline constexpr std::string_view OP_CREATE_PUBLISHER = "create_publisher";
inline constexpr std::string_view OP_CREATE_SERVICE_CLIENT =
    "create_service_client";
inline constexpr std::string_view OP_DESTROY = "destroy";

struct CreatePublisherRequest {
  uint32_t call_id;
  std::string name;
  std::string type;
};

struct CreateSubscriptionRequest {
  uint32_t call_id;
  std::string name;
  std::string type;
};

struct CreateServiceClientRequest {
  uint32_t call_id;
  std::string name;
  std::string type;
};

struct DestroyRequest {
  uint32_t id;
};

inline std::variant<std::monostate, CreatePublisherRequest,
                    CreateSubscriptionRequest, CreateServiceClientRequest,
                    DestroyRequest>
parse_text_payload(const std::string &payload) {
  try {
    nlohmann::json root = nlohmann::json::parse(payload);
    std::string op = root.at("op").get<std::string>();
    if (op == OP_CREATE_PUBLISHER) {
      return CreatePublisherRequest{root.at("callId").get<uint32_t>(),
                                    root.at("name").get<std::string>(),
                                    root.at("type").get<std::string>()};
    }
    if (op == OP_CREATE_SUBSCRIPTION) {
      return CreateSubscriptionRequest{root.at("callId").get<uint32_t>(),
                                       root.at("name").get<std::string>(),
                                       root.at("type").get<std::string>()};
    }
    if (op == OP_CREATE_SERVICE_CLIENT) {
      return CreateServiceClientRequest{root.at("callId").get<uint32_t>(),
                                        root.at("name").get<std::string>(),
                                        root.at("type").get<std::string>()};
    }
    if (op == OP_DESTROY) {
      return DestroyRequest{root.at("id").get<uint32_t>()};
    }
  } catch (const nlohmann::json::exception &) {
    // noop
  }
  return std::monostate{};
}

} // namespace ros_cdr_bridge
