#include "rby1_tools/camera_model.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rby1_tools {

Intrinsics read_intrinsics(const std::string & path) {
  YAML::Node file;
  try {
    file = YAML::LoadFile(path);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot read intrinsics file " + path + ": " + error.what());
  }
  Intrinsics result;
  try {
    const auto matrix = file["camera_matrix"];
    if (!matrix) throw std::runtime_error("no camera_matrix");
    if (matrix.IsMap()) {  // ROS calibration
      result.width = file["image_width"].as<int>();
      result.height = file["image_height"].as<int>();
      const auto data = matrix["data"].as<std::vector<double>>();
      if (data.size() != 9) throw std::runtime_error("camera_matrix.data needs 9 values");
      std::copy(data.begin(), data.end(), result.k.begin());
      if (file["distortion_coefficients"]) {
        result.d = file["distortion_coefficients"]["data"].as<std::vector<double>>();
      }
      if (file["distortion_model"]) result.model = file["distortion_model"].as<std::string>();
    } else {  // camera_ws: nested rows
      result.width = file["width"].as<int>();
      result.height = file["height"].as<int>();
      const auto rows = matrix.as<std::vector<std::vector<double>>>();
      if (rows.size() != 3 || rows[0].size() != 3 || rows[1].size() != 3 || rows[2].size() != 3) {
        throw std::runtime_error("camera_matrix must be 3x3");
      }
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) result.k[r * 3 + c] = rows[r][c];
      }
      if (file["dist_coeffs"]) result.d = file["dist_coeffs"].as<std::vector<double>>();
    }
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("intrinsics file " + path + ": " + error.what());
  } catch (const std::runtime_error & error) {
    throw std::runtime_error("intrinsics file " + path + ": " + error.what());
  }
  if (result.width <= 0 || result.height <= 0 || !(result.k[0] > 0.0) || !(result.k[4] > 0.0)) {
    throw std::runtime_error("intrinsics file " + path + ": image size and focal lengths must be positive");
  }
  result.d.resize(std::max<size_t>(result.d.size(), 5), 0.0);
  return result;
}

sensor_msgs::msg::CameraInfo camera_info(const Intrinsics & intrinsics, int width, int height,
                                         const std::string & frame_id) {
  const double sx = static_cast<double>(width) / intrinsics.width;
  const double sy = static_cast<double>(height) / intrinsics.height;
  if (std::abs(sx - sy) > 1e-3 * std::max(sx, sy)) {
    throw std::runtime_error("the intrinsics are for " + std::to_string(intrinsics.width) + "x" +
                             std::to_string(intrinsics.height) + ", the image is " + std::to_string(width) + "x" +
                             std::to_string(height) + " (another aspect ratio): calibrate at this size");
  }
  const auto & k = intrinsics.k;
  const double fx = k[0] * sx, fy = k[4] * sy, cx = k[2] * sx, cy = k[5] * sy, skew = k[1] * sx;
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(width);
  info.height = static_cast<uint32_t>(height);
  info.distortion_model = intrinsics.model;
  info.d = intrinsics.d;
  info.k = {fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {fx, skew, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

namespace {

const std::string & checked_name(const std::string & camera_name) {
  const bool valid = !camera_name.empty() && std::isalpha(static_cast<unsigned char>(camera_name[0])) &&
                     std::all_of(camera_name.begin(), camera_name.end(),
                                 [](unsigned char c) { return std::isalnum(c) || c == '_'; });
  if (!valid) {
    throw std::invalid_argument("camera_name '" + camera_name + "' must start with a letter and hold only letters, "
                                "digits and _ (it names topics and frames)");
  }
  return camera_name;
}

}  // namespace

std::string stream_topic(const std::string & camera_name, const std::string & stream) {
  return "/" + checked_name(camera_name) + "/" + stream;
}

std::string point_cloud_topic(const std::string & camera_name) {
  return "/" + checked_name(camera_name) + "/point_cloud";
}

std::string frame_name(const std::string & camera_name, const std::string & frame) {
  return checked_name(camera_name) + "_" + frame;
}

Mount read_mount(const std::string & path, const std::string & name) {
  YAML::Node file;
  try {
    file = YAML::LoadFile(path);
  } catch (const std::exception & error) {
    throw std::runtime_error("cannot read mount file " + path + ": " + error.what());
  }
  if (!file.IsMap() || !file[name]) {
    std::string names;
    if (file.IsMap()) {
      for (const auto & entry : file) names += (names.empty() ? "" : ", ") + entry.first.as<std::string>();
    }
    throw std::runtime_error("no mount '" + name + "' in " + path + " (it has: " + (names.empty() ? "none" : names) +
                             ")");
  }
  Mount mount;
  try {
    const auto entry = file[name];
    mount.parent = entry["parent"].as<std::string>();
    const auto xyz = entry["xyz"].as<std::vector<double>>();
    const auto rpy = entry["rpy"].as<std::vector<double>>();
    if (mount.parent.empty() || xyz.size() != 3 || rpy.size() != 3) {
      throw std::runtime_error("needs a parent link, xyz: [x, y, z] and rpy: [roll, pitch, yaw]");
    }
    std::copy(xyz.begin(), xyz.end(), mount.xyz.begin());
    std::copy(rpy.begin(), rpy.end(), mount.rpy.begin());
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("mount '" + name + "' in " + path + ": " + error.what());
  } catch (const std::runtime_error & error) {
    throw std::runtime_error("mount '" + name + "' in " + path + ": " + error.what());
  }
  return mount;
}

}  // namespace rby1_tools
