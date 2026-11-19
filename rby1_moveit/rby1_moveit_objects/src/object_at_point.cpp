// object_at_point: one obstacle in the MoveIt planning scene, put where a topic says.
//
//   ros2 launch rby1_moveit_objects point_object.launch.py [config:=/path/point_object.yaml]
//   ros2 topic pub --once /rby1/scene/object_point geometry_msgs/msg/PointStamped
//     "{header: {frame_id: base}, point: {x: 0.5, y: -0.3, z: 1.0}}"   (one command line)
//
// The object -- name, kind, dimensions -- is given as parameters (config/point_object.yaml).
// Every point moves it there: the same name is added again, which replaces it. An empty
// frame_id means base. The object stays in the scene when this ends
// (ros2 run rby1_moveit_objects scene remove NAME).
//
// With the parameter pose_topic it also follows a pose (geometry_msgs/PoseStamped), such as
// a marker's /rby1/marker_7/pose: the pose's position is taken to base with TF (at the
// pose's stamp) and the object goes there, when that is more than min_move from where the
// last pose put it. The pose's orientation is not used.
//   ros2 launch rby1_moveit_objects point_object.launch.py pose_topic:=/rby1/marker_7/pose
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rby1_moveit_objects/point_object.hpp"
#include "rby1_moveit_objects/scene.hpp"

namespace {

const char kBase[] = "base";
const double kTfWait = 0.2;  // s a pose waits for the TF of its stamp

}  // namespace

int main(int argc, char ** argv) {
  using namespace rby1_moveit_objects;
  using geometry_msgs::msg::PointStamped;
  using geometry_msgs::msg::PoseStamped;
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rby1_object_at_point");

  PointObject object;
  std::string pose_topic;
  double min_move = 0.0, remove_after = 0.0;
  try {
    pose_topic = node->declare_parameter("pose_topic", std::string());
    min_move = node->declare_parameter("min_move", 0.01);
    if (!(min_move >= 0.0) || !std::isfinite(min_move)) {
      throw std::invalid_argument("min_move must be 0 or more (m), got " + std::to_string(min_move));
    }
    remove_after = node->declare_parameter("remove_after", 0.0);
    if (!(remove_after >= 0.0) || !std::isfinite(remove_after)) {
      throw std::invalid_argument("remove_after must be 0 (never) or more (s), got " + std::to_string(remove_after));
    }
    ObjectSpec spec;
    spec.name = node->declare_parameter("name", std::string());
    spec.type = node->declare_parameter("kind", std::string());
    spec.size = node->declare_parameter("size", std::vector<double>{});
    spec.rpy = node->declare_parameter("rpy", std::vector<double>{0.0, 0.0, 0.0});
    const auto offset = node->declare_parameter("offset", std::vector<double>{0.0, 0.0, 0.0});
    // Without a default: left out, they stay unset and a shape that needs them is refused.
    node->declare_parameter("radius", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("height", rclcpp::PARAMETER_DOUBLE);
    double value = 0.0;
    if (node->get_parameter("radius", value)) spec.radius = value;
    if (node->get_parameter("height", value)) spec.height = value;
    object = point_object(spec, offset);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "object_at_point: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  // Only the newest point is kept: the object is put where it was last said to be.
  std::optional<PointStamped> latest;
  auto subscription = node->create_subscription<PointStamped>(
    "/rby1/scene/object_point", 10, [&latest](const PointStamped & point) { latest = point; });
  // The same for a pose: the newest one.
  std::optional<PoseStamped> latest_pose;
  std::optional<std::array<double, 3>> last_pose_place;
  auto last_pose_time = std::chrono::steady_clock::now();  // when a pose last told where it is in base
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscription;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  if (!pose_topic.empty()) {
    buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    listener = std::make_unique<tf2_ros::TransformListener>(*buffer);  // its own node and thread
    pose_subscription = node->create_subscription<PoseStamped>(
      pose_topic, rclcpp::SensorDataQoS(), [&latest_pose](const PoseStamped & pose) { latest_pose = pose; });
  }
  Scene scene(node);
  rclcpp::Clock clock(RCL_STEADY_TIME);
  RCLCPP_INFO(node->get_logger(), "%s (%s) goes where /rby1/scene/object_point says", object.spec.name.c_str(),
              object.spec.type.c_str());
  if (!pose_topic.empty()) {
    RCLCPP_INFO(node->get_logger(), "and where %s is, in %s by TF, when that is more than %.3f m from where the "
                "last pose put it", pose_topic.c_str(), kBase, min_move);
    if (remove_after > 0.0) {
      RCLCPP_INFO(node->get_logger(), "after %.1f s without a pose the object is taken out of the scene", remove_after);
    }
  }

  // Scene spins the node itself while it waits for move_group, so the node is not left
  // spinning in an executor here.
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    const bool available = scene.available();
    if (!available) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), clock, 5000,
                           "/apply_planning_scene is not available -- is move_group running on this ROS domain?");
    }
    const double since_pose =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_pose_time).count();
    if (available && last_pose_place && !latest_pose && gone(since_pose, remove_after)) {
      try {
        scene.remove({object.spec.name});
        RCLCPP_INFO(node->get_logger(), "%s removed: no pose on %s for %.1f s", object.spec.name.c_str(),
                    pose_topic.c_str(), since_pose);
      } catch (const std::exception & error) {
        RCLCPP_WARN(node->get_logger(), "%s not removed: %s", object.spec.name.c_str(), error.what());
      }
      last_pose_place.reset();  // the next pose puts it back
    }
    if (!available || (!latest && !latest_pose)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    if (latest) {
      const PointStamped point = *latest;
      latest.reset();
      try {
        const auto spec = at_point(object, point.header.frame_id, point.point.x, point.point.y, point.point.z);
        scene.apply({collision_object(spec)});
        RCLCPP_INFO(node->get_logger(), "%s (%s) at [%.3f, %.3f, %.3f] in %s", spec.name.c_str(), spec.type.c_str(),
                    spec.xyz[0], spec.xyz[1], spec.xyz[2], spec.frame.c_str());
      } catch (const std::exception & error) {
        RCLCPP_WARN(node->get_logger(), "%s not placed: %s", object.spec.name.c_str(), error.what());
      }
    }
    if (latest_pose) {
      const PoseStamped pose = *latest_pose;
      latest_pose.reset();
      try {
        const auto transform = buffer->lookupTransform(kBase, pose.header.frame_id, rclcpp::Time(pose.header.stamp),
                                                       tf2::durationFromSec(kTfWait)).transform;
        const Eigen::Quaterniond rotation(transform.rotation.w, transform.rotation.x, transform.rotation.y,
                                          transform.rotation.z);
        const Eigen::Vector3d in_base =
          rotation * Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) +
          Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
        const std::array<double, 3> place{in_base.x(), in_base.y(), in_base.z()};
        last_pose_time = std::chrono::steady_clock::now();
        if (!moved(last_pose_place, place, min_move)) continue;
        const auto spec = at_point(object, kBase, place[0], place[1], place[2]);
        scene.apply({collision_object(spec)});
        last_pose_place = place;
        RCLCPP_INFO(node->get_logger(), "%s (%s) at [%.3f, %.3f, %.3f] in %s, from %s", spec.name.c_str(),
                    spec.type.c_str(), spec.xyz[0], spec.xyz[1], spec.xyz[2], spec.frame.c_str(), pose_topic.c_str());
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(node->get_logger(), clock, 5000, "%s: where it is in %s is unknown (%s) -- are the "
                             "robot's TF (a planner's launch) and the camera's mounting TF up?", pose_topic.c_str(),
                             kBase, error.what());
      } catch (const std::exception & error) {
        RCLCPP_WARN(node->get_logger(), "%s not placed: %s", object.spec.name.c_str(), error.what());
      }
    }
  }
  rclcpp::shutdown();
  return 0;
}
