#include <sstream>
#include <stdexcept>
#include <tuple>

#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/version.h>
#include <rosidl_typesupport_cpp/identifier.hpp>

#include "ros_cdr_bridge/generic_client.hpp"

namespace ros_cdr_bridge {

#if RCLCPP_VERSION_GTE(25, 0, 0)
static const rosidl_service_type_support_t *
get_service_typesupport_handle(const std::string &type,
                               const std::string &typesupport_identifier,
                               rcpputils::SharedLibrary &library) {
  return rclcpp::get_service_typesupport_handle(type, typesupport_identifier,
                                                library);
}
#else
static std::tuple<std::string, std::string, std::string>
extract_type_identifier(const std::string &full_type) {
  char type_separator = '/';
  auto sep_position_back = full_type.find_last_of(type_separator);
  auto sep_position_front = full_type.find_first_of(type_separator);
  if (sep_position_back == std::string::npos || sep_position_back == 0 ||
      sep_position_back == full_type.length() - 1) {
    throw std::runtime_error(
        "Message type is not of the form package/type and cannot be processed");
  }

  std::string package_name = full_type.substr(0, sep_position_front);
  std::string middle_module = "";
  if (sep_position_back - sep_position_front > 0) {
    middle_module = full_type.substr(
        sep_position_front + 1, sep_position_back - sep_position_front - 1);
  }
  std::string type_name = full_type.substr(sep_position_back + 1);

  return std::make_tuple(package_name, middle_module, type_name);
}

static const void *get_typesupport_handle_impl(
    const std::string &type, const std::string &typesupport_identifier,
    const std::string &typesupport_name, const std::string &symbol_part_name,
    const std::string &middle_module_additional,
    rcpputils::SharedLibrary &library) {
  std::string package_name;
  std::string middle_module;
  std::string type_name;
  std::tie(package_name, middle_module, type_name) =
      extract_type_identifier(type);

  if (middle_module.empty()) {
    middle_module = middle_module_additional;
  }

  auto mk_error = [&package_name, &type_name, &typesupport_name](auto reason) {
    std::stringstream rcutils_dynamic_loading_error;
    rcutils_dynamic_loading_error
        << "Something went wrong loading the typesupport library for "
        << typesupport_name << " type " << package_name << "/" << type_name
        << ". " << reason;
    return rcutils_dynamic_loading_error.str();
  };

  try {
    std::string symbol_name = typesupport_identifier + symbol_part_name +
                              package_name + "__" + middle_module + "__" +
                              type_name;
    const void *(*get_ts)() = nullptr;
    // This will throw runtime_error if the symbol was not found.
    get_ts =
        reinterpret_cast<decltype(get_ts)>(library.get_symbol(symbol_name));
    return get_ts();
  } catch (std::runtime_error &) {
    throw std::runtime_error{mk_error("Library could not be found.")};
  }
}

static const rosidl_service_type_support_t *
get_service_typesupport_handle(const std::string &type,
                               const std::string &typesupport_identifier,
                               rcpputils::SharedLibrary &library) {
  return static_cast<const rosidl_service_type_support_t *>(
      get_typesupport_handle_impl(type, typesupport_identifier, "service",
                                  "__get_service_type_support_handle__", "srv",
                                  library));
}
#endif

GenericClient::GenericClient(
    rclcpp::node_interfaces::NodeBaseInterface *node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string &name, const std::string &type,
    rcl_client_options_t &options)
    : rclcpp::ClientBase{node_base, node_graph},
      request_allocator_{type + "_Request"},
      response_allocator_{type + "_Response"},
      request_deserializer_{type + "_Request"},
      response_serializer_{type + "_Response"} {
  typesupport_lib_ = rclcpp::get_typesupport_library(
      type, rosidl_typesupport_cpp::typesupport_identifier);
  typesupport_handle_ = get_service_typesupport_handle(
      type, rosidl_typesupport_cpp::typesupport_identifier, *typesupport_lib_);

  rcl_ret_t ret =
      rcl_client_init(get_client_handle().get(), get_rcl_node_handle(),
                      typesupport_handle_, name.c_str(), &options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "could not create client");
  }
}

std::shared_ptr<void> GenericClient::create_response() {
  return response_allocator_.allocate();
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header() {
  return std::make_shared<rmw_request_id_t>();
}

void GenericClient::handle_response(
    std::shared_ptr<rmw_request_id_t> request_header,
    std::shared_ptr<void> response) {
  rclcpp::SerializedMessage ser_response;
  auto ret = response_serializer_.serialize(response, ser_response);
  if (ret != RCL_RET_OK) {
    return;
  }

  std::function<void(const uint8_t *, size_t)> callback;
  {
    std::lock_guard lock{callbacks_mutex_};
    auto it = callbacks_.find(request_header->sequence_number);
    if (it == callbacks_.end()) {
      return;
    }
    callback = std::move(it->second);
    callbacks_.erase(it);
  }
  callback(ser_response.get_rcl_serialized_message().buffer,
           ser_response.get_rcl_serialized_message().buffer_length);
}

void GenericClient::async_send_request(
    const rclcpp::SerializedMessage &request,
    std::function<void(const uint8_t *, size_t)> &&callback) {
  rcl_ret_t ret;
  auto buf = request_allocator_.allocate();
  ret = request_deserializer_.deserialize(request, buf);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret,
                                             "failed to deserialize request");
  }

  std::lock_guard lock{callbacks_mutex_};
  int64_t sequence_number;
  ret =
      rcl_send_request(get_client_handle().get(), buf.get(), &sequence_number);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request");
  }
  callbacks_.emplace(sequence_number, std::move(callback));
}

} // namespace ros_cdr_bridge
