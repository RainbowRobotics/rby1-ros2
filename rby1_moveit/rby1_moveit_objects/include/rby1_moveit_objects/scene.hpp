// The MoveIt planning scene of whichever move_group runs on this ROS domain -- on
// this machine, in a container, or elsewhere on the network. Changes go through
// /apply_planning_scene, which answers only once move_group has applied them, so a
// call that returns means the scene has them.
#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rby1_moveit_objects {

class Scene {
public:
  explicit Scene(rclcpp::Node::SharedPtr node,
                 std::chrono::milliseconds timeout = std::chrono::seconds(10));

  // Whether /apply_planning_scene is there right now; does not wait.
  bool available() const;
  void apply(const std::vector<moveit_msgs::msg::CollisionObject> & objects);
  // Any planning-scene change (is_diff is set): world objects, attached objects, ...
  void apply_diff(moveit_msgs::msg::PlanningScene diff);
  // The parts of the current scene named by `components` (PlanningSceneComponents bits).
  moveit_msgs::msg::PlanningScene current(uint32_t components);
  std::vector<std::string> names();
  void remove(const std::vector<std::string> & names);
  // The object `name` as move_group holds it; std::invalid_argument if there is none.
  moveit_msgs::msg::CollisionObject get(const std::string & name);
  // Slide `name` at `velocity` (m/s, in its frame) for `duration` s at `rate` Hz, then
  // leave it where it ends.
  void move(const std::string & name, const std::array<double, 3> & velocity, double duration,
            double rate = 10.0);

private:
  template<typename Service>
  typename Service::Response::SharedPtr call(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const typename Service::Request::SharedPtr & request, const std::string & name);

  rclcpp::Node::SharedPtr node_;
  std::chrono::milliseconds timeout_;
  // One client per service, kept: a fresh one per call has to be discovered again,
  // which is too slow for moving an object ten times a second.
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_client_;
};

}  // namespace rby1_moveit_objects
