#include "rby1_moveit_objects/point_object.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rby1_moveit_objects {

PointObject point_object(const ObjectSpec & spec, const std::vector<double> & offset) {
  // The parameter is called kind; the other messages of collision_object() fit as they are.
  if (spec.type != "box" && spec.type != "sphere" && spec.type != "cylinder") {
    throw std::invalid_argument("kind must be box, sphere or cylinder, got '" + spec.type + "'");
  }
  if (offset.size() != 3 || !std::all_of(offset.begin(), offset.end(), [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("offset needs 3 finite values");
  }
  PointObject object;
  object.spec = spec;
  object.offset = {offset[0], offset[1], offset[2]};
  collision_object(at_point(object, "", 0.0, 0.0, 0.0));
  return object;
}

ObjectSpec at_point(const PointObject & object, const std::string & frame, double x, double y, double z) {
  ObjectSpec spec = object.spec;
  spec.frame = frame.empty() ? "base" : frame;
  spec.xyz = {x + object.offset[0], y + object.offset[1], z + object.offset[2]};
  return spec;
}

bool moved(const std::optional<std::array<double, 3>> & last, const std::array<double, 3> & now, double min_move) {
  if (!last) return true;
  return std::hypot((*last)[0] - now[0], (*last)[1] - now[1], (*last)[2] - now[2]) > min_move;
}

bool gone(double since_last_pose, double remove_after) {
  return remove_after > 0.0 && since_last_pose > remove_after;
}

}  // namespace rby1_moveit_objects
