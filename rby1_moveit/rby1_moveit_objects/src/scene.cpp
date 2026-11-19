#include "rby1_moveit_objects/scene.hpp"

#include <cmath>
#include <stdexcept>
#include <thread>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

#include "rby1_moveit_objects/object_spec.hpp"

namespace rby1_moveit_objects {

using moveit_msgs::msg::CollisionObject;
using moveit_msgs::srv::ApplyPlanningScene;
using moveit_msgs::srv::GetPlanningScene;

Scene::Scene(rclcpp::Node::SharedPtr node, std::chrono::milliseconds timeout)
: node_(std::move(node)), timeout_(timeout),
  apply_client_(node_->create_client<ApplyPlanningScene>("/apply_planning_scene")),
  get_client_(node_->create_client<GetPlanningScene>("/get_planning_scene")) {}

template<typename Service>
typename Service::Response::SharedPtr Scene::call(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  const typename Service::Request::SharedPtr & request, const std::string & name) {
  if (!client->wait_for_service(timeout_)) {
    throw std::runtime_error(name + " unavailable -- is move_group running on this ROS domain?");
  }
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future, timeout_) != rclcpp::FutureReturnCode::SUCCESS) {
    throw std::runtime_error(name + " did not answer");
  }
  return future.get();
}

bool Scene::available() const { return apply_client_->service_is_ready(); }

void Scene::apply(const std::vector<CollisionObject> & objects) {
  moveit_msgs::msg::PlanningScene diff;
  diff.world.collision_objects = objects;
  apply_diff(diff);
}

void Scene::apply_diff(moveit_msgs::msg::PlanningScene diff) {
  auto request = std::make_shared<ApplyPlanningScene::Request>();
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  request->scene = diff;
  auto response = call<ApplyPlanningScene>(apply_client_, request, "/apply_planning_scene");
  if (!response->success) throw std::runtime_error("move_group refused the planning scene update");
}

moveit_msgs::msg::PlanningScene Scene::current(uint32_t components) {
  auto request = std::make_shared<GetPlanningScene::Request>();
  request->components.components = components;
  return call<GetPlanningScene>(get_client_, request, "/get_planning_scene")->scene;
}

std::vector<std::string> Scene::names() {
  auto request = std::make_shared<GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES;
  auto response = call<GetPlanningScene>(get_client_, request, "/get_planning_scene");
  std::vector<std::string> names;
  for (const auto & object : response->scene.world.collision_objects) names.push_back(object.id);
  return names;
}

void Scene::remove(const std::vector<std::string> & names) {
  std::vector<CollisionObject> objects;
  for (const auto & name : names) {
    CollisionObject object;
    object.id = name;
    object.operation = CollisionObject::REMOVE;
    objects.push_back(object);
  }
  apply(objects);
}

CollisionObject Scene::get(const std::string & name) {
  auto request = std::make_shared<GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  auto response = call<GetPlanningScene>(get_client_, request, "/get_planning_scene");
  for (const auto & object : response->scene.world.collision_objects) {
    if (object.id == name) return object;
  }
  throw std::invalid_argument("no object named '" + name + "' in the scene (scene list)");
}

void Scene::move(const std::string & name, const std::array<double, 3> & velocity, double duration,
                 double rate) {
  check_move({velocity[0], velocity[1], velocity[2]}, duration, rate);
  CollisionObject object = get(name);
  object.operation = CollisionObject::ADD;  // re-adding an object replaces it where it is
  const std::array<double, 3> start{object.pose.position.x, object.pose.position.y, object.pose.position.z};
  const auto began = std::chrono::steady_clock::now();
  while (true) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    const auto at = moved(start, velocity, elapsed, duration);
    object.pose.position.x = at[0];
    object.pose.position.y = at[1];
    object.pose.position.z = at[2];
    apply({object});
    if (elapsed >= duration) return;
    const double next = (std::floor(elapsed * rate) + 1) / rate;
    std::this_thread::sleep_until(began + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double>(next)));
  }
}

}  // namespace rby1_moveit_objects
