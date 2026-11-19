// Obstacles as plain specs -- the form a command line or a YAML file gives them --
// and their MoveIt CollisionObject. Model independent: a pose is in `frame`
// (default base, the root link of every RB-Y1).
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>

namespace rby1_moveit_objects {

struct ObjectSpec {
  std::string name;
  std::string type;           // box, sphere or cylinder
  std::string frame = "base";
  std::vector<double> xyz;    // centre (m), 3 values; required
  std::vector<double> rpy{0.0, 0.0, 0.0};  // orientation (rad)
  std::vector<double> size;   // box: 3 values (m)
  std::optional<double> radius;  // sphere, cylinder (m)
  std::optional<double> height;  // cylinder (m)
};

// (x, y, z, w) of Rz(yaw) * Ry(pitch) * Rx(roll).
std::array<double, 4> quaternion_from_rpy(double roll, double pitch, double yaw);

// The CollisionObject (operation ADD) for `spec`; std::invalid_argument saying what is
// wrong with it otherwise.
moveit_msgs::msg::CollisionObject collision_object(const ObjectSpec & spec);

// The `objects:` list of a YAML file (see config/example_scene.yaml), each entry
// checked by collision_object(); std::invalid_argument naming the file otherwise.
std::vector<ObjectSpec> specs_from_file(const std::string & path);

// Where an object that started at `start` is `elapsed` s into a `duration` s slide.
std::array<double, 3> moved(
  const std::array<double, 3> & start, const std::array<double, 3> & velocity,
  double elapsed, double duration);

// std::invalid_argument unless a slide is sensible.
void check_move(const std::vector<double> & velocity, double duration, double rate);

}  // namespace rby1_moveit_objects
