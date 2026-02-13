#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <variant>

#include <nlohmann/json.hpp>

#include <rmw/qos_profiles.h>
#include <rmw/types.h>

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
  rmw_qos_profile_t qos_profile;
};

struct CreateSubscriptionRequest {
  uint32_t call_id;
  std::string name;
  std::string type;
  rmw_qos_profile_t qos_profile;
};

struct CreateServiceClientRequest {
  uint32_t call_id;
  std::string name;
  std::string type;
  rmw_qos_profile_t qos_profile;
};

struct DestroyRequest {
  uint32_t id;
};

inline rmw_qos_profile_t parse_qos_base_profile(const std::string &value) {
  if (value == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  }
  if (value == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  }
  if (value == "DEFAULT") {
    return rmw_qos_profile_default;
  }
  if (value == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  }
  if (value == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  }
  if (value == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  }
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
  if (value == "BEST_AVAILABLE") {
    return rmw_qos_profile_best_available;
  }
#endif
  return rmw_qos_profile_unknown;
}

inline rmw_qos_history_policy_t parse_history_policy(const std::string &value) {
  if (value == "SYSTEM_DEFAULT") {
    return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  }
  if (value == "KEEP_LAST") {
    return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if (value == "KEEP_ALL") {
    return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  }
  return RMW_QOS_POLICY_HISTORY_UNKNOWN;
}

inline rmw_qos_reliability_policy_t
parse_reliability_policy(const std::string &value) {
  if (value == "SYSTEM_DEFAULT") {
    return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  }
  if (value == "RELIABLE") {
    return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (value == "BEST_EFFORT") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
  if (value == "BEST_AVAILABLE") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
}

inline rmw_qos_durability_policy_t
parse_durability_policy(const std::string &value) {
  if (value == "SYSTEM_DEFAULT") {
    return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  }
  if (value == "TRANSIENT_LOCAL") {
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  if (value == "VOLATILE") {
    return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
#ifdef RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE
  if (value == "BEST_AVAILABLE") {
    return RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_DURABILITY_UNKNOWN;
}

inline rmw_qos_liveliness_policy_t
parse_liveliness_policy(const std::string &value) {
  if (value == "SYSTEM_DEFAULT") {
    return RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  }
  if (value == "AUTOMATIC") {
    return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  if (value == "MANUAL_BY_TOPIC") {
    return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  }
#ifdef RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE
  if (value == "BEST_AVAILABLE") {
    return RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

inline rmw_time_t parse_rmw_time(const nlohmann::json &data) {
  rmw_time_t time;
  time.sec = data.at("sec").get<uint64_t>();
  time.nsec = data.at("nsec").get<uint64_t>();
  return time;
}

inline rmw_qos_profile_t parse_rmw_qos_profile(const nlohmann::json &data,
                                               bool is_service = false) {
  rmw_qos_profile_t profile = rmw_qos_profile_default;
  if (is_service) {
    profile = rmw_qos_profile_services_default;
  }
  if (data.contains("profile")) {
    profile = parse_qos_base_profile(data.at("profile").get<std::string>());
  }
  if (data.contains("history")) {
    profile.history =
        parse_history_policy(data.at("history").get<std::string>());
  }
  if (data.contains("depth")) {
    profile.depth = data.at("depth").get<size_t>();
  }
  if (data.contains("reliability")) {
    profile.reliability =
        parse_reliability_policy(data.at("reliability").get<std::string>());
  }
  if (data.contains("durability")) {
    profile.durability =
        parse_durability_policy(data.at("durability").get<std::string>());
  }
  if (data.contains("deadline")) {
    profile.deadline = parse_rmw_time(data.at("deadline"));
  }
  if (data.contains("lifespan")) {
    profile.lifespan = parse_rmw_time(data.at("lifespan"));
  }
  if (data.contains("liveliness")) {
    profile.liveliness =
        parse_liveliness_policy(data.at("liveliness").get<std::string>());
  }
  if (data.contains("livelinessLeaseDuration")) {
    profile.liveliness_lease_duration =
        parse_rmw_time(data.at("livelinessLeaseDuration"));
  }
  if (data.contains("avoidRosNamespaceConventions")) {
    profile.avoid_ros_namespace_conventions =
        data.at("avoidRosNamespaceConventions").get<bool>();
  }
  return profile;
}

inline std::variant<std::monostate, CreatePublisherRequest,
                    CreateSubscriptionRequest, CreateServiceClientRequest,
                    DestroyRequest>
parse_text_payload(const std::string &payload) {
  try {
    nlohmann::json data = nlohmann::json::parse(payload);
    std::string op = data.at("op").get<std::string>();
    if (op == OP_CREATE_PUBLISHER) {
      return CreatePublisherRequest{data.at("callId").get<uint32_t>(),
                                    data.at("name").get<std::string>(),
                                    data.at("type").get<std::string>(),
                                    parse_rmw_qos_profile(data.at("qos"))};
    }
    if (op == OP_CREATE_SUBSCRIPTION) {
      return CreateSubscriptionRequest{data.at("callId").get<uint32_t>(),
                                       data.at("name").get<std::string>(),
                                       data.at("type").get<std::string>(),
                                       parse_rmw_qos_profile(data.at("qos"))};
    }
    if (op == OP_CREATE_SERVICE_CLIENT) {
      return CreateServiceClientRequest{
          data.at("callId").get<uint32_t>(), data.at("name").get<std::string>(),
          data.at("type").get<std::string>(),
          parse_rmw_qos_profile(data.at("qos"), true)};
    }
    if (op == OP_DESTROY) {
      return DestroyRequest{data.at("id").get<uint32_t>()};
    }
  } catch (const nlohmann::json::exception &) {
    // noop
  }
  return std::monostate{};
}

} // namespace ros_cdr_bridge
