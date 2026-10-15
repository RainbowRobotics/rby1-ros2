#pragma once

// Small helpers that hide ros2_control API differences between Humble and Jazzy.

#include <limits>
#include <string>
#include <vector>

#if __has_include("realtime_tools/realtime_buffer.hpp")
#include "realtime_tools/realtime_buffer.hpp"
#else
#include "realtime_tools/realtime_buffer.h"
#endif

#if __has_include("realtime_tools/realtime_publisher.hpp")
#include "realtime_tools/realtime_publisher.hpp"
#else
#include "realtime_tools/realtime_publisher.h"
#endif

#if __has_include("hardware_interface/types/hardware_component_interface_params.hpp")
#define RBY1_CONTROLLERS_OPTIONAL_HANDLE_API 1
#endif

namespace rby1_controllers
{

template <typename Handle>
double read_value(const Handle & handle)
{
#ifdef RBY1_CONTROLLERS_OPTIONAL_HANDLE_API
  return handle.get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
#else
  return handle.get_value();
#endif
}

template <typename Handle>
void write_value(Handle & handle, double value)
{
  (void)handle.set_value(value);
}

// Index of the loaned interface with full name `name`, or -1.
template <typename Loaned>
int find_interface(const std::vector<Loaned> & interfaces, const std::string & name)
{
  for (size_t i = 0; i < interfaces.size(); ++i) {
    if (interfaces[i].get_name() == name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace rby1_controllers
