#include "rby1_moveit_objects/object_spec.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace rby1_moveit_objects {

namespace {

bool all_finite(const std::vector<double> & values) {
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

std::string list(const std::vector<double> & values) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < values.size(); ++i) out << (i ? ", " : "") << values[i];
  out << "]";
  return out.str();
}

std::vector<double> numbers(const YAML::Node & node, const std::string & what) {
  if (!node.IsSequence()) throw std::invalid_argument(what + " must be a list of numbers");
  std::vector<double> values;
  for (const auto & item : node) values.push_back(item.as<double>());
  return values;
}

}  // namespace

std::array<double, 4> quaternion_from_rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

moveit_msgs::msg::CollisionObject collision_object(const ObjectSpec & spec) {
  using shape_msgs::msg::SolidPrimitive;
  const std::string & name = spec.name;
  if (spec.type != "box" && spec.type != "sphere" && spec.type != "cylinder") {
    throw std::invalid_argument("'" + name + "': type must be one of ['box', 'cylinder', 'sphere']");
  }
  if (name.empty()) throw std::invalid_argument("every object needs a name");

  SolidPrimitive primitive;
  if (spec.type == "box") {
    if (spec.size.empty()) throw std::invalid_argument(name + ": a box needs size");
    if (spec.size.size() != 3) throw std::invalid_argument(name + ": box size needs 3 values");
    primitive.type = SolidPrimitive::BOX;
    primitive.dimensions.assign(spec.size.begin(), spec.size.end());
  } else if (spec.type == "sphere") {
    if (!spec.radius) throw std::invalid_argument(name + ": a sphere needs radius");
    primitive.type = SolidPrimitive::SPHERE;
    primitive.dimensions.push_back(*spec.radius);
  } else {
    std::vector<std::string> missing;
    if (!spec.height) missing.push_back("height");
    if (!spec.radius) missing.push_back("radius");
    if (!missing.empty()) {
      std::string text = missing[0];
      for (size_t i = 1; i < missing.size(); ++i) text += ", " + missing[i];
      throw std::invalid_argument(name + ": a cylinder needs " + text);
    }
    primitive.type = SolidPrimitive::CYLINDER;
    primitive.dimensions.push_back(*spec.height);  // MoveIt's order: height, radius
    primitive.dimensions.push_back(*spec.radius);
  }
  const auto & dims = primitive.dimensions;
  if (!std::all_of(dims.begin(), dims.end(), [](double d) { return std::isfinite(d) && d > 0; })) {
    throw std::invalid_argument(name + ": dimensions must be positive, got " +
                                list(std::vector<double>(dims.begin(), dims.end())));
  }
  if (spec.xyz.empty()) throw std::invalid_argument(name + ": needs xyz");
  if (spec.xyz.size() != 3 || spec.rpy.size() != 3 || !all_finite(spec.xyz) || !all_finite(spec.rpy)) {
    throw std::invalid_argument(name + ": xyz and rpy need 3 finite values each");
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = spec.xyz[0];
  pose.position.y = spec.xyz[1];
  pose.position.z = spec.xyz[2];
  const auto q = quaternion_from_rpy(spec.rpy[0], spec.rpy[1], spec.rpy[2]);
  pose.orientation.x = q[0];
  pose.orientation.y = q[1];
  pose.orientation.z = q[2];
  pose.orientation.w = q[3];

  moveit_msgs::msg::CollisionObject object;
  object.id = name;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.header.frame_id = spec.frame;
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  return object;
}

std::vector<ObjectSpec> specs_from_file(const std::string & path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument(path + ": " + error.what());
  }
  const YAML::Node entries = root["objects"];
  if (!entries || !entries.IsSequence() || entries.size() == 0) {
    throw std::invalid_argument(path + ": expected a non-empty \"objects:\" list");
  }
  std::vector<ObjectSpec> specs;
  for (const auto & entry : entries) {
    ObjectSpec spec;
    try {
      if (entry["name"]) spec.name = entry["name"].as<std::string>();
      if (entry["type"]) spec.type = entry["type"].as<std::string>();
      if (entry["frame"]) spec.frame = entry["frame"].as<std::string>();
      if (entry["xyz"]) spec.xyz = numbers(entry["xyz"], "xyz");
      if (entry["rpy"]) spec.rpy = numbers(entry["rpy"], "rpy");
      if (entry["size"]) spec.size = numbers(entry["size"], "size");
      if (entry["radius"]) spec.radius = entry["radius"].as<double>();
      if (entry["height"]) spec.height = entry["height"].as<double>();
    } catch (const YAML::Exception & error) {
      throw std::invalid_argument(path + ": object '" + spec.name + "': " + error.what());
    }
    collision_object(spec);  // refuse the whole file for one bad entry
    specs.push_back(spec);
  }
  return specs;
}

std::array<double, 3> moved(
  const std::array<double, 3> & start, const std::array<double, 3> & velocity,
  double elapsed, double duration) {
  const double t = std::min(std::max(elapsed, 0.0), duration);
  return {start[0] + velocity[0] * t, start[1] + velocity[1] * t, start[2] + velocity[2] * t};
}

void check_move(const std::vector<double> & velocity, double duration, double rate) {
  if (velocity.size() != 3 || !all_finite(velocity)) {
    throw std::invalid_argument("velocity needs 3 finite values (m/s)");
  }
  if (!(std::isfinite(duration) && duration > 0 && duration <= 600)) {
    throw std::invalid_argument("time must be in (0, 600] s");
  }
  if (!(rate >= 1 && rate <= 50)) throw std::invalid_argument("rate must be in [1, 50] Hz");
}

}  // namespace rby1_moveit_objects
