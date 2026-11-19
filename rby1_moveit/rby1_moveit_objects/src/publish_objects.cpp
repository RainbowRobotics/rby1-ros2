// publish_objects: put the modules of a YAML file into the MoveIt planning scene and
// keep them there.
//
//   ros2 run rby1_moveit_objects publish_objects --ros-args -p config:=/path/objects.yaml
//   ros2 launch rby1_moveit_objects objects.launch.py [config:=...]
//
// Every `check_period` s it looks whether they are all still in the scene (a restarted
// move_group starts empty) and puts them back if not. On Ctrl+C they are taken away
// again (remove_on_exit), so modules exist only while this runs.
//
// A module with `follows:` (a gripper finger) is moved to where its joint of the given
// sensor_msgs/JointState topic is, at most every `follow_period` s: when it has moved more
// than `follow_tolerance` m, and wherever it has come to rest.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <map>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "rby1_moveit_objects/modules.hpp"
#include "rby1_moveit_objects/scene.hpp"

namespace {
std::atomic<bool> stop{false};
}  // namespace

int main(int argc, char ** argv) {
  using namespace rby1_moveit_objects;
  using moveit_msgs::msg::PlanningSceneComponents;
  // Own the Ctrl+C: the modules must still be taken away through ROS afterwards.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, [](int) { stop = true; });
  std::signal(SIGTERM, [](int) { stop = true; });
  auto node = rclcpp::Node::make_shared("rby1_moveit_objects");
  const auto config = node->declare_parameter(
    "config", ament_index_cpp::get_package_share_directory("rby1_moveit_objects") + "/config/objects.yaml");
  const double period = node->declare_parameter("check_period", 2.0);
  const bool remove_on_exit = node->declare_parameter("remove_on_exit", true);
  const double follow_period = node->declare_parameter("follow_period", 0.1);
  const double follow_tolerance = node->declare_parameter("follow_tolerance", 0.001);

  int code = 0;
  std::vector<Module> modules;
  Scene scene(node);
  try {
    modules = modules_from_file(config);
    // Where each following module is in the scene, and where its topic last said.
    std::vector<double> applied(modules.size(), 0.0), latest(modules.size(), 0.0), seen(modules.size(), 0.0);
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> follow_topics;
    for (size_t i = 0; i < modules.size(); ++i) {
      if (!modules[i].follows) continue;
      applied[i] = latest[i] = seen[i] = modules[i].follows->initial;
      const auto & topic = modules[i].follows->topic;
      if (follow_topics.count(topic)) continue;
      follow_topics[topic] = node->create_subscription<sensor_msgs::msg::JointState>(
        topic, 10, [&modules, &latest, topic](const sensor_msgs::msg::JointState & state) {
          for (size_t m = 0; m < modules.size(); ++m) {
            if (!modules[m].follows || modules[m].follows->topic != topic) continue;
            for (size_t j = 0; j < state.name.size() && j < state.position.size(); ++j) {
              if (state.name[j] == modules[m].follows->joint && std::isfinite(state.position[j])) {
                latest[m] = state.position[j];
              }
            }
          }
        });
      RCLCPP_INFO(node->get_logger(), "following %s", topic.c_str());
    }
    const auto placed = [&modules, &applied] {
      std::vector<Module> now;
      for (size_t i = 0; i < modules.size(); ++i) now.push_back(at_position(modules[i], applied[i]));
      return now;
    };
    // The RB-Y1 v1.3 carries its gripper in its URDF (links gripper_right / gripper_left):
    // the gripper modules of rby1_tools' gripper.yaml would put a second one on its hands.
    const auto links = scene.current(PlanningSceneComponents::ALLOWED_COLLISION_MATRIX)
                         .allowed_collision_matrix.entry_names;
    const auto has_link = [&links](const std::string & name) {
      return std::find(links.begin(), links.end(), name) != links.end();
    };
    if (has_link("gripper_right") || has_link("gripper_left")) {
      for (const auto & m : modules) {
        if (m.spec.name.rfind("gripper_", 0) == 0) {
          throw std::runtime_error("the robot model already has the gripper (RB-Y1 v1.3): not loading module '" +
                                   m.spec.name + "' -- do not use rby1_tools' gripper.yaml with it");
        }
      }
    }
    scene.apply_diff(add_diff(placed()));
    std::string names;
    for (const auto & m : modules) names += (names.empty() ? "" : ", ") + m.spec.name + (m.attach ? " on " : " at ") + m.spec.frame;
    RCLCPP_INFO(node->get_logger(), "%zu object(s) from %s: %s", modules.size(), config.c_str(), names.c_str());
    auto next = std::chrono::steady_clock::now();
    auto next_follow = next;
    while (!stop) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      rclcpp::spin_some(node);
      const auto now = std::chrono::steady_clock::now();
      if (!follow_topics.empty() && now >= next_follow) {
        next_follow = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(follow_period));
        std::vector<Module> moved;
        std::string what;
        for (size_t i = 0; i < modules.size(); ++i) {
          // Far enough to be worth it, or come to rest anywhere else (the last small step).
          const bool settled = latest[i] == seen[i];
          seen[i] = latest[i];
          if (modules[i].follows && latest[i] != applied[i] &&
              (std::abs(latest[i] - applied[i]) > follow_tolerance || settled)) {
            applied[i] = latest[i];
            moved.push_back(at_position(modules[i], applied[i]));
            what += (what.empty() ? "" : ", ") + modules[i].spec.name + " " + std::to_string(applied[i]);
          }
        }
        if (!moved.empty()) {
          try {
            scene.apply_diff(add_diff(moved));  // the same names again: they replace the ones there
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "moved: %s (m)", what.c_str());
          } catch (const std::exception & error) {
            RCLCPP_WARN(node->get_logger(), "moving what follows a joint: %s", error.what());
          }
        }
      }
      if (now < next) continue;
      next = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  std::chrono::duration<double>(period));
      try {
        const auto gone = missing(modules, scene.current(PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                                         PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS));
        if (!gone.empty()) {
          RCLCPP_WARN(node->get_logger(), "%zu object(s) missing from the scene (move_group restarted?); putting "
                                          "them back", gone.size());
          scene.apply_diff(add_diff(placed()));
        }
      } catch (const std::exception & error) {
        RCLCPP_WARN(node->get_logger(), "planning scene check: %s", error.what());
      }
    }
    // Stopped together with move_group (a launch's Ctrl+C), there is nothing to remove
    // from and waiting for the service would only delay the shutdown.
    if (remove_on_exit && !scene.available()) {
      RCLCPP_INFO(node->get_logger(), "move_group is gone; nothing to remove the %zu object(s) from",
                  modules.size());
    } else if (remove_on_exit) {
      scene.apply_diff(remove_diff(modules));
      RCLCPP_INFO(node->get_logger(), "removed %zu object(s)", modules.size());
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "rby1_moveit_objects: %s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
