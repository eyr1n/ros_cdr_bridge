#pragma once

#include <cstdint>
#include <string>
#include <variant>

#include <nlohmann/json.hpp>

#include <rmw/qos_profiles.h>
#include <rmw/types.h>

namespace ros_cdr_bridge {

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
  if (value == "sensor_data") {
    return rmw_qos_profile_sensor_data;
  }
  if (value == "parameters") {
    return rmw_qos_profile_parameters;
  }
  if (value == "default") {
    return rmw_qos_profile_default;
  }
  if (value == "services_default") {
    return rmw_qos_profile_services_default;
  }
  if (value == "parameter_events") {
    return rmw_qos_profile_parameter_events;
  }
  if (value == "system_default") {
    return rmw_qos_profile_system_default;
  }
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
  if (value == "best_available") {
    return rmw_qos_profile_best_available;
  }
#endif
  return rmw_qos_profile_unknown;
}

inline rmw_qos_history_policy_t parse_history_policy(const std::string &value) {
  if (value == "system_default") {
    return RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
  }
  if (value == "keep_last") {
    return RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  }
  if (value == "keep_all") {
    return RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  }
  return RMW_QOS_POLICY_HISTORY_UNKNOWN;
}

inline rmw_qos_reliability_policy_t
parse_reliability_policy(const std::string &value) {
  if (value == "system_default") {
    return RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
  }
  if (value == "reliable") {
    return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (value == "best_effort") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  }
#ifdef RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE
  if (value == "best_available") {
    return RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
}

inline rmw_qos_durability_policy_t
parse_durability_policy(const std::string &value) {
  if (value == "system_default") {
    return RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
  }
  if (value == "transient_local") {
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  if (value == "volatile") {
    return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }
#ifdef RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE
  if (value == "best_available") {
    return RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_DURABILITY_UNKNOWN;
}

inline rmw_qos_liveliness_policy_t
parse_liveliness_policy(const std::string &value) {
  if (value == "system_default") {
    return RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
  }
  if (value == "automatic") {
    return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
  }
  if (value == "manual_by_topic") {
    return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
  }
#ifdef RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE
  if (value == "best_available") {
    return RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE;
  }
#endif
  return RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
}

inline rmw_time_t parse_rmw_time(const nlohmann::json &data) {
  if (data.is_string() && data.get<std::string>() == "infinite") {
    return RMW_DURATION_INFINITE;
  }
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
  if (data.contains("liveliness_lease_duration")) {
    profile.liveliness_lease_duration =
        parse_rmw_time(data.at("liveliness_lease_duration"));
  }
  if (data.contains("avoid_ros_namespace_conventions")) {
    profile.avoid_ros_namespace_conventions =
        data.at("avoid_ros_namespace_conventions").get<bool>();
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
    if (op == "create_publisher") {
      return CreatePublisherRequest{data.at("call_id").get<uint32_t>(),
                                    data.at("name").get<std::string>(),
                                    data.at("type").get<std::string>(),
                                    parse_rmw_qos_profile(data.at("qos"))};
    }
    if (op == "create_subscription") {
      return CreateSubscriptionRequest{data.at("call_id").get<uint32_t>(),
                                       data.at("name").get<std::string>(),
                                       data.at("type").get<std::string>(),
                                       parse_rmw_qos_profile(data.at("qos"))};
    }
    if (op == "create_service_client") {
      return CreateServiceClientRequest{
          data.at("call_id").get<uint32_t>(),
          data.at("name").get<std::string>(),
          data.at("type").get<std::string>(),
          parse_rmw_qos_profile(data.at("qos"), true)};
    }
    if (op == "destroy") {
      return DestroyRequest{data.at("id").get<uint32_t>()};
    }
  } catch (const nlohmann::json::exception &) {
    // noop
  }
  return std::monostate{};
}

} // namespace ros_cdr_bridge
