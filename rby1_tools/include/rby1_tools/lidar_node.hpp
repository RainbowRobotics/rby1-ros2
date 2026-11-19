// The LiDAR nodes as lidar_merger and lidar_calibrate see them: which are on (the launch's flags,
// passed on as bool parameters) and how each is set (asked from the running node itself, so that
// lidar.yaml is read in one place).
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rby1_tools/camera_model.hpp"

namespace rby1_tools {

// The launch flags and the node each starts: front_left -> lidar_front_left.
inline constexpr std::array<const char *, 4> kLidarPlaces{"front_left", "front_right", "rear_left", "rear_right"};

// The nodes whose flag is true (declared here as bool parameters, false when not given).
inline std::vector<std::string> lidars_on(rclcpp::Node & node) {
  std::vector<std::string> on;
  for (const char * place : kLidarPlaces) {
    if (node.declare_parameter(place, false)) on.push_back(std::string("lidar_") + place);
  }
  return on;
}

// The topic a LiDAR node publishes on: /<lidar_name>/scan, or, when its scan goes to lidar_merger
// only, /<lidar_name>/_scan (a leading _ hides it from ros2 topic list).
inline std::string lidar_scan_topic(const std::string & lidar_name, bool merge) {
  return stream_topic(lidar_name, merge ? "_scan" : "scan");
}

struct LidarNode {
  std::string node, lidar_name, parent;
  bool merge = false;
  std::array<double, 3> xyz{}, rpy{};
};

// A running LiDAR node's settings. Call before `node` spins; throws when it does not answer
// within `wait`.
inline LidarNode lidar_node_settings(const rclcpp::Node::SharedPtr & node, const std::string & name,
                                     std::chrono::seconds wait) {
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, name);
  if (!client->wait_for_service(wait)) {
    throw std::runtime_error(name + " does not answer: is it running? (its log says why not, e.g. its port taken by "
                                    "another lidar.launch.py)");
  }
  const auto values = client->get_parameters({"lidar_name", "parent", "merge", "xyz", "rpy"}, wait);
  if (values.size() != 5) throw std::runtime_error(name + " did not give its settings");
  LidarNode s;
  s.node = name;
  s.lidar_name = values[0].as_string();
  s.parent = values[1].as_string();
  s.merge = values[2].as_bool();
  const auto xyz = values[3].as_double_array(), rpy = values[4].as_double_array();
  if (xyz.size() != 3 || rpy.size() != 3) throw std::runtime_error(name + ": xyz and rpy need 3 values each");
  for (size_t i = 0; i < 3; ++i) {
    s.xyz[i] = xyz[i];
    s.rpy[i] = rpy[i];
  }
  return s;
}

}  // namespace rby1_tools
