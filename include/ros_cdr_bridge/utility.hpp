#pragma once

namespace ros_cdr_bridge {

template <class... Ts> struct overloads : Ts... {
  using Ts::operator()...;
};
template <class... Ts> overloads(Ts...) -> overloads<Ts...>;

} // namespace ros_cdr_bridge
