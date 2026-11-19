// The camera's files in the package's config/camera/, and the frames a camera hangs on the
// robot by, as static TF: <parent> -> <camera_name>_link (the mount, from camera_mount.yaml)
// and <camera_name>_link -> <camera_name>_optical_frame (z forward, x right, y down: the
// images' frame).
#pragma once

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "rby1_tools/camera_model.hpp"

namespace rby1_tools {

inline std::string camera_config_dir() {
  return ament_index_cpp::get_package_share_directory("rby1_tools") + "/config/camera";
}

// `intrinsics_file`: a file name in config/camera/intrinsics/ (its README gives the layouts),
// or an absolute path; empty stays empty (no calibration file).
inline std::string intrinsics_path(const std::string & intrinsics_file) {
  if (intrinsics_file.empty() || intrinsics_file.front() == '/') return intrinsics_file;
  return camera_config_dir() + "/intrinsics/" + intrinsics_file;
}

// `mount_name` names an entry of config/camera/camera_mount.yaml; "none" (or empty, which
// ros2 launch cannot pass) leaves <camera_name>_link without a parent.
inline bool mounted(const std::string & mount_name) { return !mount_name.empty() && mount_name != "none"; }

inline std::vector<geometry_msgs::msg::TransformStamped> mount_transforms(rclcpp::Node * node,
                                                                          const std::string & camera_name,
                                                                          const std::string & mount_name) {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  const auto link = frame_name(camera_name, "link");
  if (mounted(mount_name)) {
    const auto mount = read_mount(camera_config_dir() + "/camera_mount.yaml", mount_name);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->now();
    t.header.frame_id = mount.parent;
    t.child_frame_id = link;
    t.transform.translation.x = mount.xyz[0];
    t.transform.translation.y = mount.xyz[1];
    t.transform.translation.z = mount.xyz[2];
    tf2::Quaternion q;
    q.setRPY(mount.rpy[0], mount.rpy[1], mount.rpy[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    transforms.push_back(t);
  }
  geometry_msgs::msg::TransformStamped optical;  // roll -90 deg, yaw -90 deg
  optical.header.stamp = node->now();
  optical.header.frame_id = link;
  optical.child_frame_id = frame_name(camera_name, "optical_frame");
  optical.transform.rotation.x = -0.5;
  optical.transform.rotation.y = 0.5;
  optical.transform.rotation.z = -0.5;
  optical.transform.rotation.w = 0.5;
  transforms.push_back(optical);
  return transforms;
}

}  // namespace rby1_tools
