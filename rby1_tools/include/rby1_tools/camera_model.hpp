// Camera calibrations, and the names and mount a camera is published under.
#pragma once

#include <array>
#include <string>
#include <variant>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

namespace rby1_tools {

// A calibrated camera: 3x3 camera matrix (row-major) and plumb_bob distortion
// (k1, k2, p1, p2, k3) at the image size it was measured for.
struct Intrinsics {
  int width = 0, height = 0;
  std::array<double, 9> k{};
  std::vector<double> d;
  std::string model = "plumb_bob";
};

// An intrinsics YAML file, in either layout:
//   camera_ws (OpenCV):  width, height, camera_matrix: [[fx,0,cx],[0,fy,cy],[0,0,1]], dist_coeffs: [...]
//   ROS calibration:     image_width, image_height, camera_matrix: {data: [9]},
//                        distortion_model, distortion_coefficients: {data: [...]}
Intrinsics read_intrinsics(const std::string & path);

// The CameraInfo for an image of `width` x `height`: the camera matrix scaled if the
// file was measured at another size with the same aspect ratio (refused otherwise).
sensor_msgs::msg::CameraInfo camera_info(const Intrinsics & intrinsics, int width, int height,
                                         const std::string & frame_id);

// Names from `camera_name` (a letter, then letters, digits or _), so that several cameras
// can run side by side. A stream (rgb, depth, ir_left, ir_right) has its image on
// /<camera_name>/<stream>/raw or .../compressed and its CameraInfo on .../info.
std::string stream_topic(const std::string & camera_name, const std::string & stream);  // /<camera_name>/<stream>
std::string point_cloud_topic(const std::string & camera_name);                         // /<camera_name>/point_cloud
std::string frame_name(const std::string & camera_name, const std::string & frame);     // <camera_name>_<frame>

// Where a camera sits: <camera_name>_link at `xyz` (m) and `rpy` (rad) in `parent`.
struct Mount {
  std::string parent;
  std::array<double, 3> xyz{}, rpy{};
};

// The mount called `name` in a mount YAML (one entry per name: parent, xyz, rpy).
Mount read_mount(const std::string & path, const std::string & name);

}  // namespace rby1_tools
