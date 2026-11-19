// lidar_merger: the LiDARs' scans as one, on one frame (lidar.launch.py starts it with them). With
// `merge: true` (config/lidar/lidar.yaml, for all its nodes) each LiDAR node sends its scan on the
// hidden /<lidar_name>/_scan only; this node places those on `target_frame` by TF (each node's
// static TF from its parent, xyz and rpy) and publishes /lidar_merged/scan: 360 deg in `bins`
// sectors from -pi, each with its nearest return seen from the frame's origin, +inf where none.
// With `merge: false` it stops at once and each LiDAR keeps its own /<lidar_name>/scan.
//
// A merged scan waits for a new scan from every LiDAR on: one that goes quiet holds it (warned)
// rather than let out a scan with a side missing, which would read as open space.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "rby1_tools/lidar_model.hpp"
#include "rby1_tools/lidar_node.hpp"

namespace {

constexpr char kMergedTopic[] = "/lidar_merged/scan";

rby1_tools::ScanData scan_data(const sensor_msgs::msg::LaserScan & msg) {
  rby1_tools::ScanData scan;
  scan.angle_min = msg.angle_min;
  scan.angle_increment = msg.angle_increment;
  scan.range_min = msg.range_min;
  scan.range_max = msg.range_max;
  scan.ranges = msg.ranges;
  scan.intensities = msg.intensities;
  return scan;
}

rby1_tools::Pose3 pose_of(const geometry_msgs::msg::Transform & t) {
  const tf2::Matrix3x3 m(tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
  rby1_tools::Pose3 pose;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) pose.rotation[r * 3 + c] = m[r][c];
  }
  pose.translation = {t.translation.x, t.translation.y, t.translation.z};
  return pose;
}

}  // namespace

class LidarMerger : public rclcpp::Node {
public:
  LidarMerger() : rclcpp::Node("lidar_merger") {
    merge_ = declare_parameter("merge", false);
    target_frame_ = declare_parameter("target_frame", "base");
    bins_ = declare_parameter("bins", 1440);
    range_min_ = declare_parameter("range_min", 0.0);
    range_max_ = declare_parameter("range_max", 100.0);
    timeout_ = declare_parameter("timeout", 1.0);
    lidars_ = rby1_tools::lidars_on(*this);
  }

  // False when there is nothing to merge (merge: false). Asks the LiDAR nodes, so before spinning.
  bool start() {
    if (!merge_) {
      RCLCPP_INFO(get_logger(), "merge: false -- each LiDAR publishes its own /<lidar_name>/scan; nothing to merge");
      return false;
    }
    if (lidars_.empty()) throw std::invalid_argument("no LiDAR to merge: front_left, front_right, rear_left, rear_right "
                                                     "are all false");
    if (bins_ < 1) throw std::invalid_argument("bins must be 1 or more");
    if (timeout_ <= 0.0) throw std::invalid_argument("timeout must be above 0 s");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    // All settings first: asking spins this node, which must not merge before every input is there.
    std::vector<rby1_tools::LidarNode> settings;
    for (const auto & name : lidars_) {
      settings.push_back(rby1_tools::lidar_node_settings(shared_from_this(), name, std::chrono::seconds(10)));
      if (!settings.back().merge) {
        throw std::runtime_error(name + " has merge: false while lidar_merger has true (one lidar.yaml?)");
      }
    }
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(kMergedTopic, 10);
    started_ = now();
    std::string topics;
    for (const auto & s : settings) {
      const auto & name = s.node;
      const auto topic = rby1_tools::lidar_scan_topic(s.lidar_name, true);
      const size_t i = inputs_.size();
      inputs_.push_back({name, nullptr, false, started_});
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
          topic, 10, [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(i, std::move(msg)); }));
      topics += (topics.empty() ? "" : ", ") + topic;
    }
    watchdog_ = create_wall_timer(std::chrono::seconds(1), [this] { warn_quiet(); });
    RCLCPP_INFO(get_logger(), "merging %s on %s into %s (%ld sectors)", topics.c_str(), target_frame_.c_str(),
                kMergedTopic, bins_);
    return true;
  }

private:
  struct Input {
    std::string node;
    sensor_msgs::msg::LaserScan::ConstSharedPtr latest;
    bool fresh = false;  // came after the last merged scan
    rclcpp::Time received;
  };

  void on_scan(size_t i, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    inputs_[i].latest = std::move(msg);
    inputs_[i].fresh = true;
    inputs_[i].received = now();
    if (std::all_of(inputs_.begin(), inputs_.end(), [](const Input & in) { return in.fresh; })) publish();
  }

  void publish() {
    std::vector<rby1_tools::Point2> points;
    rclcpp::Time newest(0, 0, get_clock()->get_clock_type());
    float scan_time = 0.0f;
    for (const auto & in : inputs_) {
      const auto & msg = *in.latest;
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(target_frame_, msg.header.frame_id, tf2::TimePointZero);
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s is not placed on %s (%s): no merged scan; give it "
                             "parent, xyz and rpy in lidar.yaml", msg.header.frame_id.c_str(), target_frame_.c_str(),
                             e.what());
        return;
      }
      const auto placed = rby1_tools::scan_points(scan_data(msg), pose_of(t.transform));
      points.insert(points.end(), placed.begin(), placed.end());
      const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
      if (stamp > newest) newest = stamp;
      scan_time = std::max(scan_time, msg.scan_time);
    }
    sensor_msgs::msg::LaserScan out;
    out.header.stamp = newest;
    out.header.frame_id = target_frame_;
    out.angle_min = static_cast<float>(-M_PI);
    out.angle_increment = static_cast<float>(2.0 * M_PI / static_cast<double>(bins_));
    out.angle_max = out.angle_min + out.angle_increment * static_cast<float>(bins_ - 1);
    out.time_increment = 0.0f;  // put together from several sensors: no single sweep
    out.scan_time = scan_time;
    out.range_min = static_cast<float>(range_min_);
    out.range_max = static_cast<float>(range_max_);
    rby1_tools::merge_points(points, static_cast<size_t>(bins_), range_min_, range_max_, out.ranges, out.intensities);
    pub_->publish(out);
    for (auto & in : inputs_) in.fresh = false;
  }

  void warn_quiet() {
    const auto t = now();
    std::string quiet;  // all in one line: a throttled warning per sensor would drop all but one
    for (const auto & in : inputs_) {
      const double s = (t - in.received).seconds();
      if (s > timeout_) quiet += (quiet.empty() ? "" : ", ") + in.node + " for " + std::to_string(std::lround(s)) + " s";
    }
    if (!quiet.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no scan from %s: %s is held until every LiDAR sends again",
                           quiet.c_str(), kMergedTopic);
    }
  }

  bool merge_ = false;
  std::string target_frame_;
  long bins_ = 1440;
  double range_min_ = 0.0, range_max_ = 100.0, timeout_ = 1.0;
  std::vector<std::string> lidars_;
  rclcpp::Time started_;
  std::vector<Input> inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<LidarMerger>();
    if (node->start()) rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_merger"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
