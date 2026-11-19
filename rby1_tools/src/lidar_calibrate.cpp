// lidar_calibrate: how far off lidar.yaml places each LiDAR against a reference one, from what they
// see in common. Run once by lidar_calibrate.launch.py, which starts the LiDARs as lidar.yaml says
// and stops them when this ends; prints the xyz and rpy to put in lidar.yaml and changes no file.
//
// Each LiDAR's points are placed by its settings (parent, xyz, rpy) and matched to the reference's
// (point-to-line ICP on the plane) over `scan_pairs` scans; the mean and spread are printed. On the
// plane only: x, y and yaw change, z, roll and pitch stay. The reference stays as set: where they
// all sit on the robot is not seen this way, so measure that one.
// Nothing is offered for a LiDAR when the surroundings do not pin x, y and yaw (a corridor, one
// wall: stand where walls in several directions are in view), when it sees too little in common
// with the reference, or when the correction is past max_correction / max_correction_deg (then the
// settings are wrong, not a little off).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rby1_tools/lidar_model.hpp"
#include "rby1_tools/lidar_node.hpp"

namespace {

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

struct Spread {
  double mean = 0.0, sd = 0.0;
};

Spread spread(const std::vector<double> & v) {
  Spread s;
  for (const double x : v) s.mean += x;
  s.mean /= static_cast<double>(v.size());
  for (const double x : v) s.sd += (x - s.mean) * (x - s.mean);
  s.sd = v.size() > 1 ? std::sqrt(s.sd / static_cast<double>(v.size() - 1)) : 0.0;
  return s;
}

}  // namespace

class LidarCalibrate : public rclcpp::Node {
public:
  LidarCalibrate() : rclcpp::Node("lidar_calibrate") {
    reference_ = declare_parameter("reference", "lidar_front_left");
    scan_pairs_ = declare_parameter("scan_pairs", 20);
    timeout_ = declare_parameter("timeout", 15.0);
    max_correction_ = declare_parameter("max_correction", 0.10);
    max_correction_deg_ = declare_parameter("max_correction_deg", 5.0);
    lidars_ = rby1_tools::lidars_on(*this);
  }

  double timeout() const { return timeout_; }
  bool collected() const { return sets_.size() >= static_cast<size_t>(scan_pairs_); }
  size_t sets() const { return sets_.size(); }

  // Asks the LiDAR nodes their settings, so before spinning.
  void start() {
    if (scan_pairs_ < 2) throw std::invalid_argument("scan_pairs must be 2 or more");
    if (std::find(lidars_.begin(), lidars_.end(), reference_) == lidars_.end()) {
      throw std::invalid_argument("the reference " + reference_ + " is not among the LiDARs started");
    }
    if (lidars_.size() < 2) throw std::invalid_argument("one LiDAR only: nothing to match against " + reference_);
    for (const auto & name : lidars_) {
      settings_.push_back(rby1_tools::lidar_node_settings(shared_from_this(), name, std::chrono::seconds(10)));
      if (settings_.back().parent.empty() || settings_.back().parent != settings_.front().parent) {
        throw std::invalid_argument("the LiDARs must all be placed on one parent (" + name + ": '" +
                                    settings_.back().parent + "')");
      }
    }
    latest_.resize(settings_.size());
    for (size_t i = 0; i < settings_.size(); ++i) {
      const auto topic = rby1_tools::lidar_scan_topic(settings_[i].lidar_name, settings_[i].merge);
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
          topic, 10, [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(i, std::move(msg)); }));
    }
    RCLCPP_INFO(get_logger(), "collecting %ld scans from each LiDAR; keep the robot and its surroundings still",
                scan_pairs_);
  }

  // Prints each LiDAR's result; true when every one has a value to put in lidar.yaml.
  bool report() const {
    const size_t ref = index_of(reference_);
    const auto & base = settings_[ref];
    std::printf("\nreference: %s (kept)  xyz: [%.5f, %.5f, %.5f]  rpy: [%.6f, %.6f, %.6f]\n", base.node.c_str(),
                base.xyz[0], base.xyz[1], base.xyz[2], base.rpy[0], base.rpy[1], base.rpy[2]);
    const auto ref_pose = rby1_tools::pose_from_xyz_rpy(base.xyz, base.rpy);
    bool all = true;
    for (size_t k = 0; k < settings_.size(); ++k) {
      if (k == ref) continue;
      const auto & s = settings_[k];
      const auto pose = rby1_tools::pose_from_xyz_rpy(s.xyz, s.rpy);
      std::vector<double> dx, dy, dyaw, rms;
      size_t degenerate = 0, unconverged = 0;
      for (const auto & set : sets_) {
        const auto a = rby1_tools::align_points(rby1_tools::scan_points(set[ref], ref_pose),
                                                rby1_tools::scan_points(set[k], pose));
        if (!a.converged) {
          ++unconverged;
          continue;
        }
        if (a.degenerate) {
          ++degenerate;
          continue;
        }
        const auto & c = a.correction;  // moves the sensor's place on the parent's plane too
        const double cc = std::cos(c.yaw), cs = std::sin(c.yaw);
        dx.push_back(cc * s.xyz[0] - cs * s.xyz[1] + c.x - s.xyz[0]);
        dy.push_back(cs * s.xyz[0] + cc * s.xyz[1] + c.y - s.xyz[1]);
        dyaw.push_back(c.yaw);
        rms.push_back(a.rms);
      }
      std::printf("%s: %zu of %zu scan pairs matched", s.node.c_str(), dx.size(), sets_.size());
      if (degenerate + unconverged > 0) std::printf(" (%zu not pinned down, %zu not converged)", degenerate, unconverged);
      std::printf("\n");
      if (dx.size() * 2 < sets_.size()) {
        std::printf("  nothing offered: too few pairs matched. Stand where walls in several directions are in view of "
                    "both (not a corridor, not one wall) and run again\n");
        all = false;
        continue;
      }
      const auto mx = spread(dx), my = spread(dy), myaw = spread(dyaw);
      std::printf("  correction  dx %+.4f ± %.4f m  dy %+.4f ± %.4f m  dyaw %+.3f ± %.3f deg  (residual %.4f m)\n",
                  mx.mean, mx.sd, my.mean, my.sd, myaw.mean * 180.0 / M_PI, myaw.sd * 180.0 / M_PI,
                  spread(rms).mean);
      if (std::hypot(mx.mean, my.mean) > max_correction_ || std::abs(myaw.mean) * 180.0 / M_PI > max_correction_deg_) {
        std::printf("  nothing offered: past the limit (%.3f m, %.1f deg) -- the settings are wrong, not a little off "
                    "(the right section? the right sensor_ip and port?)\n",
                    max_correction_, max_correction_deg_);
        all = false;
        continue;
      }
      rby1_tools::Pose3 fixed = pose;
      const double cc = std::cos(myaw.mean), cs = std::sin(myaw.mean);
      for (int c = 0; c < 3; ++c) {  // Rz(dyaw) * rotation
        const double r0 = pose.rotation[c], r1 = pose.rotation[3 + c];
        fixed.rotation[c] = cc * r0 - cs * r1;
        fixed.rotation[3 + c] = cs * r0 + cc * r1;
      }
      const auto rpy = rby1_tools::rpy_of(fixed);
      std::printf("  -> lidar.yaml %s:  xyz: [%.5f, %.5f, %.5f]  rpy: [%.6f, %.6f, %.6f]\n", s.node.c_str(),
                  s.xyz[0] + mx.mean, s.xyz[1] + my.mean, s.xyz[2], rpy[0], rpy[1], rpy[2]);
    }
    std::fflush(stdout);
    return all;
  }

private:
  size_t index_of(const std::string & node) const {
    for (size_t i = 0; i < settings_.size(); ++i) {
      if (settings_[i].node == node) return i;
    }
    throw std::logic_error("no " + node);
  }

  // A set is taken on each reference scan that every other LiDAR has a scan just before.
  void on_scan(size_t i, sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    latest_[i] = {scan_data(*msg), now(), true};
    if (settings_[i].node != reference_ || collected()) return;
    const auto t = now();
    std::vector<rby1_tools::ScanData> set;
    for (const auto & l : latest_) {
      if (!l.got || (t - l.received).seconds() > 0.2) return;
      set.push_back(l.scan);
    }
    sets_.push_back(std::move(set));
  }

  struct Latest {
    rby1_tools::ScanData scan;
    rclcpp::Time received;
    bool got = false;
  };

  std::string reference_;
  long scan_pairs_ = 20;
  double timeout_ = 15.0, max_correction_ = 0.10, max_correction_deg_ = 5.0;
  std::vector<std::string> lidars_;
  std::vector<rby1_tools::LidarNode> settings_;
  std::vector<Latest> latest_;
  std::vector<std::vector<rby1_tools::ScanData>> sets_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    auto node = std::make_shared<LidarCalibrate>();
    node->start();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(node->timeout());
    while (rclcpp::ok() && !node->collected() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    if (!node->collected()) {
      RCLCPP_ERROR(node->get_logger(), "got %zu scan sets in %.0f s, not enough: is every LiDAR sending? (lidar.launch.py "
                   "still running on their ports?)", node->sets(), node->timeout());
    } else if (node->report()) {
      code = 0;
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_calibrate"), "%s", error.what());
  }
  rclcpp::shutdown();
  return code;
}
