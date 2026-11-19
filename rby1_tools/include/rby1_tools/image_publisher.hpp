// An image topic as raw or as JPEG/PNG (image_transport's compressed plugin), one of the two.
#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rby1_tools {

// For a stream topic (rby1_tools::stream_topic, /<camera_name>/<stream>): `compressed` false,
// sensor_msgs/Image on <stream>/raw; true, sensor_msgs/CompressedImage on <stream>/compressed
// only -- the name image_transport gives the compressed form of <stream> -- as `format`
// ("jpeg" for 8-bit images; "png", lossless, for 16-bit depth, which JPEG cannot take),
// encoded only while something subscribes. Reliable, as Isaac ROS subscribes reliably.
inline image_transport::Publisher image_publisher(rclcpp::Node * node, const std::string & stream, bool compressed,
                                                  const std::string & format = "jpeg") {
  const auto topic = compressed ? stream : stream + "/raw";
  // image_transport offers every transport it finds unless <topic>.enable_pub_plugins
  // says otherwise; the names are built the way image_transport builds them.
  const auto resolved = rclcpp::expand_topic_or_service_name(topic, node->get_name(), node->get_namespace());
  auto base = resolved.substr(node->get_effective_namespace().length());
  std::replace(base.begin(), base.end(), '/', '.');
  // Both are set before the publisher is made: the plugin reads its format only then.
  const auto plugins = base + ".enable_pub_plugins";
  const std::vector<std::string> wanted{compressed ? "image_transport/compressed" : "image_transport/raw"};
  node->declare_parameter(plugins, wanted);
  if (compressed) node->declare_parameter(base + ".format", format);
  auto qos = rmw_qos_profile_default;  // reliable
  qos.depth = 5;
  auto publisher = image_transport::create_publisher(node, resolved, qos);
  if (node->get_parameter(plugins).as_string_array() != wanted ||
      (compressed && node->get_parameter(base + ".format").as_string() != format)) {
    throw std::runtime_error("image_transport did not take " + plugins + (compressed ? " or .format" : ""));
  }
  return publisher;
}

// The topic `image_publisher` publishes on, for the log.
inline std::string image_topic(const std::string & stream, bool compressed) {
  return stream + (compressed ? "/compressed" : "/raw");
}

}  // namespace rby1_tools
