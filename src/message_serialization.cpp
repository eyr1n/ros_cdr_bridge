#include <cstring>

#include <rclcpp/typesupport_helpers.hpp>
#include <rclcpp/version.h>
#include <rmw/rmw.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_cpp/message_type_support_dispatch.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "ros_cdr_bridge/message_serialization.hpp"

namespace ros_cdr_bridge {

static const rosidl_message_type_support_t *
get_message_typesupport_handle(const std::string &type,
                               const std::string &typesupport_identifier,
                               rcpputils::SharedLibrary &library) {
#if RCLCPP_VERSION_GTE(25, 0, 0)
  return rclcpp::get_message_typesupport_handle(type, typesupport_identifier,
                                                library);
#else
  return rclcpp::get_typesupport_handle(type, typesupport_identifier, library);
#endif
}

MessageAllocator::MessageAllocator(const std::string &type) {
  typesupport_lib_ = rclcpp::get_typesupport_library(
      type, rosidl_typesupport_cpp::typesupport_identifier);
  typesupport_handle_ = get_message_typesupport_handle(
      type, rosidl_typesupport_cpp::typesupport_identifier, *typesupport_lib_);
  introspection_handle_ =
      rosidl_typesupport_cpp::get_message_typesupport_handle_function(
          typesupport_handle_,
          rosidl_typesupport_introspection_cpp::typesupport_identifier);
}

std::shared_ptr<void> MessageAllocator::allocate() {
  auto members =
      static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          introspection_handle_->data);
  void *buf = ::operator new(members->size_of_);
  std::memset(buf, 0, members->size_of_);
  members->init_function(buf, rosidl_runtime_cpp::MessageInitialization::ALL);
  return std::shared_ptr<void>(
      buf, [members, typesupport_lib = typesupport_lib_](void *buf) {
        members->fini_function(buf);
        ::operator delete(buf);
      });
}

MessageSerializer::MessageSerializer(const std::string &type) {
  typesupport_lib_ = rclcpp::get_typesupport_library(
      type, rosidl_typesupport_cpp::typesupport_identifier);
  typesupport_handle_ = get_message_typesupport_handle(
      type, rosidl_typesupport_cpp::typesupport_identifier, *typesupport_lib_);
}

rmw_ret_t MessageSerializer::serialize(const std::shared_ptr<void> src,
                                       rclcpp::SerializedMessage &dest) {
  return rmw_serialize(src.get(), typesupport_handle_,
                       &dest.get_rcl_serialized_message());
}

MessageDeserializer::MessageDeserializer(const std::string &type) {
  typesupport_lib_ = rclcpp::get_typesupport_library(
      type, rosidl_typesupport_cpp::typesupport_identifier);
  typesupport_handle_ = get_message_typesupport_handle(
      type, rosidl_typesupport_cpp::typesupport_identifier, *typesupport_lib_);
}

rmw_ret_t MessageDeserializer::deserialize(const rclcpp::SerializedMessage &src,
                                           std::shared_ptr<void> dest) {
  return rmw_deserialize(&src.get_rcl_serialized_message(), typesupport_handle_,
                         dest.get());
}

} // namespace ros_cdr_bridge
