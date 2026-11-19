// `ros2 run rby1_moveit_objects scene ...`: obstacles in the MoveIt planning scene.
// Usage: scene --help. Prints what it did, or "scene: <reason>" and exits non-zero.
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rby1_moveit_objects/command.hpp"
#include "rby1_moveit_objects/object_spec.hpp"
#include "rby1_moveit_objects/scene.hpp"

namespace {

// 0.7 -> "0.7", 0 -> "0.0": metres as the old Python tool printed them.
std::string metres(double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.3f", value);
  std::string result(text);
  while (result.size() > 1 && result.back() == '0' && result[result.size() - 2] != '.') result.pop_back();
  return result;
}

}  // namespace

int main(int argc, char ** argv) {
  using namespace rby1_moveit_objects;
  const auto all = rclcpp::remove_ros_arguments(argc, argv);
  const std::vector<std::string> args(all.begin() + 1, all.end());
  Command command;
  try {
    command = parse_command(args);
  } catch (const std::invalid_argument & error) {
    std::cerr << "scene: " << error.what() << "\n\n" << USAGE;
    return 2;
  }
  if (command.name == "help") {
    std::cout << USAGE;
    return 0;
  }

  rclcpp::init(argc, argv);
  int code = 0;
  try {
    Scene scene(rclcpp::Node::make_shared("rby1_scene"));
    if (command.name == "add") {
      scene.apply({collision_object(command.spec)});
      std::cout << "added " << command.spec.type << " " << command.spec.name << std::endl;
    } else if (command.name == "move") {
      const std::array<double, 3> velocity{command.velocity[0], command.velocity[1], command.velocity[2]};
      scene.move(command.names[0], velocity, command.time, command.rate);
      char time[32];
      std::snprintf(time, sizeof(time), "%g", command.time);
      std::cout << "moved " << command.names[0] << " by [" << metres(velocity[0] * command.time) << ", "
                << metres(velocity[1] * command.time) << ", " << metres(velocity[2] * command.time)
                << "] m over " << time << " s" << std::endl;
    } else if (command.name == "remove") {
      scene.remove(command.names);
      std::string names;
      for (const auto & name : command.names) names += (names.empty() ? "" : " ") + name;
      std::cout << "removed " << names << std::endl;
    } else if (command.name == "clear") {
      const auto names = scene.names();
      if (!names.empty()) scene.remove(names);
      std::cout << "cleared " << names.size() << " object(s)" << std::endl;
    } else if (command.name == "list") {
      for (const auto & name : scene.names()) std::cout << name << std::endl;
    } else if (command.name == "load") {
      std::vector<moveit_msgs::msg::CollisionObject> objects;
      for (const auto & spec : specs_from_file(command.file)) objects.push_back(collision_object(spec));
      scene.apply(objects);
      std::cout << "loaded " << objects.size() << " object(s) from " << command.file << std::endl;
    }
  } catch (const std::exception & error) {
    std::cerr << "scene: " << error.what() << std::endl;
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
