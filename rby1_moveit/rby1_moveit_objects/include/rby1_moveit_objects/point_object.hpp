// The one object the object_at_point node places: its shape as the node's parameters
// give it (config/point_object.yaml), and the ObjectSpec it becomes at a point.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "rby1_moveit_objects/object_spec.hpp"

namespace rby1_moveit_objects {

struct PointObject {
  ObjectSpec spec;  // name, type, rpy, size / radius / height; xyz and frame come with each point
  std::array<double, 3> offset{0.0, 0.0, 0.0};  // added to the point (m, in the point's frame)
};

// `spec` (its xyz and frame are not used) and `offset`, checked as collision_object()
// checks an object; std::invalid_argument saying what is wrong otherwise.
PointObject point_object(const ObjectSpec & spec, const std::vector<double> & offset);

// The object with its centre at the point plus the offset, in `frame` (empty: base).
ObjectSpec at_point(const PointObject & object, const std::string & frame, double x, double y, double z);

// Whether a place read from the pose topic is worth moving the object for: none was used
// yet, or it is more than `min_move` (m) from the last one used. A marker's pose shakes a
// little in every image, and every move of the object is a change of the planning scene.
bool moved(const std::optional<std::array<double, 3>> & last, const std::array<double, 3> & now, double min_move);

// Whether the object a pose placed is to be taken out of the scene: `remove_after` is
// set (above 0) and no pose came for longer than that (both in seconds).
bool gone(double since_last_pose, double remove_after);

}  // namespace rby1_moveit_objects
