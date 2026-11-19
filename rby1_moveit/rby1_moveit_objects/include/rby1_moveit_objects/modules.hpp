// Modules on the robot, and fixtures around it, as MoveIt planning-scene objects.
//
// Each entry of a YAML file (config/objects.yaml) names a frame -- a robot link such as
// ee_right or link_head_2, or base -- a pose in that frame, and a shape: a mesh file or
// a box, sphere or cylinder. `attach: true` fixes it to the link so it moves with the
// arm (an attached object: shown in RViz, and part of the arm for MoveIt's collision
// checking); `attach: false` places it in the world where the frame is when applied.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "rby1_moveit_objects/object_spec.hpp"

namespace rby1_moveit_objects {

// A module that moves with one joint of a sensor_msgs/JointState topic -- a gripper finger
// of rby1_tools' gripper_driver: its xyz shifts by axis * the joint's position.
struct Follow {
  std::string topic;
  std::string joint;
  std::array<double, 3> axis{0.0, 0.0, 0.0};
  double initial = 0.0;  // the position until the topic gives one
};

struct Module {
  ObjectSpec spec;   // name, frame, xyz, rpy; type box/sphere/cylinder/mesh
  std::string mesh;  // resolved URI for type mesh: file://..., package://...
  std::array<double, 3> scale{1.0, 1.0, 1.0};
  bool attach = true;
  std::vector<std::string> touch_links;  // attached: links it may touch; default its own link
  std::optional<Follow> follows;         // `follows:` -- xyz is then the pose at position 0
};

// `module` where its joint is at `position`: xyz + follows.axis * position. A module
// that follows nothing comes back as it is.
Module at_position(const Module & module, double position);

// The `objects:` list of a YAML file. A relative mesh path is taken from the file's
// folder. std::invalid_argument naming the file and entry otherwise.
std::vector<Module> modules_from_file(const std::string & path);

// The CollisionObject (ADD) of one module, the mesh loaded; std::invalid_argument if
// the mesh cannot be read.
moveit_msgs::msg::CollisionObject collision_object(const Module & module);

// A planning-scene diff that adds every module: world objects, and attached objects on
// their links.
moveit_msgs::msg::PlanningScene add_diff(const std::vector<Module> & modules);

// A diff that takes them all away again -- attached ones detached and removed.
moveit_msgs::msg::PlanningScene remove_diff(const std::vector<Module> & modules);

// Names of `modules` missing from `scene` (world objects and attached objects).
std::vector<std::string> missing(const std::vector<Module> & modules, const moveit_msgs::msg::PlanningScene & scene);

}  // namespace rby1_moveit_objects
