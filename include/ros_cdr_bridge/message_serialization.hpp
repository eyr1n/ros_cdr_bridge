#pragma once

#include <memory>
#include <string>

#include <rclcpp/serialized_message.hpp>
#include <rcpputils/shared_library.hpp>
#include <rmw/types.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace ros_cdr_bridge {

class MessageAllocator {
public:
  MessageAllocator(const std::string &type);
  std::shared_ptr<void> allocate();

private:
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib_;
  const rosidl_message_type_support_t *typesupport_handle_;
  const rosidl_message_type_support_t *introspection_handle_;
};

class MessageSerializer {
public:
  MessageSerializer(const std::string &type);
  rmw_ret_t serialize(const std::shared_ptr<void> src,
                      rclcpp::SerializedMessage &dest);

private:
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib_;
  const rosidl_message_type_support_t *typesupport_handle_;
};

class MessageDeserializer {
public:
  MessageDeserializer(const std::string &type);
  rmw_ret_t deserialize(const rclcpp::SerializedMessage &src,
                        std::shared_ptr<void> dest);

private:
  std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib_;
  const rosidl_message_type_support_t *typesupport_handle_;
};

} // namespace ros_cdr_bridge
