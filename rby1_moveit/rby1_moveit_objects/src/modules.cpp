#include "rby1_moveit_objects/modules.hpp"

#include <algorithm>
#include <filesystem>
#include <memory>
#include <set>
#include <stdexcept>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <yaml-cpp/yaml.h>

namespace rby1_moveit_objects {

namespace fs = std::filesystem;
using moveit_msgs::msg::AttachedCollisionObject;
using moveit_msgs::msg::CollisionObject;

namespace {

std::vector<double> numbers(const YAML::Node & node, const std::string & what) {
  if (!node.IsSequence()) throw std::invalid_argument(what + " must be a list of numbers");
  std::vector<double> values;
  for (const auto & item : node) values.push_back(item.as<double>());
  return values;
}

std::string resolve(const std::string & mesh, const fs::path & folder) {
  if (mesh.rfind("package://", 0) == 0 || mesh.rfind("file://", 0) == 0) return mesh;
  const fs::path path = fs::path(mesh).is_absolute() ? fs::path(mesh) : folder / mesh;
  if (!fs::exists(path)) throw std::invalid_argument("mesh file not found: " + path.string());
  return "file://" + fs::weakly_canonical(path).string();
}

}  // namespace

std::vector<Module> modules_from_file(const std::string & path) {
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
  const fs::path folder = fs::absolute(fs::path(path)).parent_path();
  std::vector<Module> modules;
  std::set<std::string> names;
  for (const auto & entry : entries) {
    Module module;
    auto & spec = module.spec;
    try {
      if (entry["name"]) spec.name = entry["name"].as<std::string>();
      if (entry["type"]) spec.type = entry["type"].as<std::string>();
      if (!entry["frame"]) throw std::invalid_argument("needs frame (a robot link such as ee_right, or base)");
      spec.frame = entry["frame"].as<std::string>();
      if (entry["xyz"]) spec.xyz = numbers(entry["xyz"], "xyz");
      else spec.xyz = {0.0, 0.0, 0.0};
      if (entry["rpy"]) spec.rpy = numbers(entry["rpy"], "rpy");
      if (entry["size"]) spec.size = numbers(entry["size"], "size");
      if (entry["radius"]) spec.radius = entry["radius"].as<double>();
      if (entry["height"]) spec.height = entry["height"].as<double>();
      if (entry["attach"]) module.attach = entry["attach"].as<bool>();
      if (entry["touch_links"]) {
        for (const auto & link : entry["touch_links"]) module.touch_links.push_back(link.as<std::string>());
      }
      if (entry["scale"]) {
        const auto scale = numbers(entry["scale"], "scale");
        if (scale.size() != 3) throw std::invalid_argument("scale needs 3 values");
        module.scale = {scale[0], scale[1], scale[2]};
      }
      if (spec.type == "mesh") {
        if (!entry["mesh"]) throw std::invalid_argument("a mesh needs mesh: <file>");
        module.mesh = resolve(entry["mesh"].as<std::string>(), folder);
      }
      if (const auto follows = entry["follows"]) {
        Follow follow;
        if (!follows["topic"] || !follows["joint"] || !follows["axis"]) {
          throw std::invalid_argument("follows needs topic, joint and axis");
        }
        follow.topic = follows["topic"].as<std::string>();
        follow.joint = follows["joint"].as<std::string>();
        const auto axis = numbers(follows["axis"], "follows: axis");
        if (axis.size() != 3 || std::all_of(axis.begin(), axis.end(), [](double v) { return v == 0.0; })) {
          throw std::invalid_argument("follows: axis needs 3 values, not all 0");
        }
        follow.axis = {axis[0], axis[1], axis[2]};
        if (follows["initial"]) follow.initial = follows["initial"].as<double>();
        if (!module.attach) throw std::invalid_argument("follows is for attached modules (attach: true)");
        module.follows = follow;
      }
      if (spec.name.empty()) throw std::invalid_argument("every object needs a name");
      if (!names.insert(spec.name).second) throw std::invalid_argument("the name is used twice");
      if (module.touch_links.empty()) module.touch_links = {spec.frame};
      collision_object(module);  // refuse the whole file for one bad entry
    } catch (const YAML::Exception & error) {
      throw std::invalid_argument(path + ": object '" + spec.name + "': " + error.what());
    } catch (const std::invalid_argument & error) {
      throw std::invalid_argument(path + ": object '" + spec.name + "': " + error.what());
    }
    modules.push_back(module);
  }
  return modules;
}

Module at_position(const Module & module, double position) {
  if (!module.follows) return module;
  Module moved = module;
  for (size_t i = 0; i < 3; ++i) moved.spec.xyz[i] += module.follows->axis[i] * position;
  return moved;
}

CollisionObject collision_object(const Module & module) {
  if (module.spec.type != "mesh") return collision_object(module.spec);

  // A box stands in for the mesh to get the frame and pose checked and built.
  auto stand_in = module.spec;
  stand_in.type = "box";
  stand_in.size = {1.0, 1.0, 1.0};
  CollisionObject object = collision_object(stand_in);
  object.primitives.clear();
  const auto pose = object.primitive_poses.front();
  object.primitive_poses.clear();

  if (!std::all_of(module.scale.begin(), module.scale.end(), [](double s) { return s > 0; })) {
    throw std::invalid_argument("scale must be positive");
  }
  std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
    module.mesh, Eigen::Vector3d(module.scale[0], module.scale[1], module.scale[2])));
  if (!mesh || mesh->triangle_count == 0) throw std::invalid_argument("cannot read mesh " + module.mesh);
  shapes::ShapeMsg message;
  shapes::constructMsgFromShape(mesh.get(), message);
  object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(message));
  object.mesh_poses.push_back(pose);
  return object;
}

moveit_msgs::msg::PlanningScene add_diff(const std::vector<Module> & modules) {
  moveit_msgs::msg::PlanningScene diff;
  for (const auto & module : modules) {
    auto object = collision_object(module);
    if (module.attach) {
      AttachedCollisionObject attached;
      attached.link_name = module.spec.frame;
      attached.object = object;
      attached.touch_links = module.touch_links;
      diff.robot_state.attached_collision_objects.push_back(attached);
    } else {
      diff.world.collision_objects.push_back(object);
    }
  }
  return diff;
}

moveit_msgs::msg::PlanningScene remove_diff(const std::vector<Module> & modules) {
  moveit_msgs::msg::PlanningScene diff;
  for (const auto & module : modules) {
    CollisionObject gone;
    gone.id = module.spec.name;
    gone.operation = CollisionObject::REMOVE;
    if (module.attach) {
      // Detaching leaves the object in the world; remove it there as well.
      AttachedCollisionObject detached;
      detached.link_name = module.spec.frame;
      detached.object = gone;
      diff.robot_state.attached_collision_objects.push_back(detached);
    }
    diff.world.collision_objects.push_back(gone);
  }
  return diff;
}

std::vector<std::string> missing(const std::vector<Module> & modules, const moveit_msgs::msg::PlanningScene & scene) {
  std::set<std::string> present;
  for (const auto & object : scene.world.collision_objects) present.insert(object.id);
  for (const auto & attached : scene.robot_state.attached_collision_objects) present.insert(attached.object.id);
  std::vector<std::string> gone;
  for (const auto & module : modules) {
    if (!present.count(module.spec.name)) gone.push_back(module.spec.name);
  }
  return gone;
}

}  // namespace rby1_moveit_objects
