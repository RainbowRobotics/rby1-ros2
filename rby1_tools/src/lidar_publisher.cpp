// lidar_publisher: a Richbeam LakiBeam1 2D LiDAR as a ROS topic. The sensor sends its scan as UDP
// packets to a host and port it keeps (its web API: /api/v1/sensor/overview, "host"); this node
// listens on that port and publishes each full turn. Settings: config/lidar/lidar.yaml.
//
//   ros2 launch rby1_tools lidar.launch.py            # left and right, on the PC the sensors send to
//
// Publishes /<lidar_name>/scan (sensor_msgs/LaserScan, in <lidar_name>_link: angle -pi at the
// sensor's azimuth 0, rising with it, as the vendor's driver lays it out; with `merge: true` on the
// hidden /<lidar_name>/_scan instead, for lidar_merger only) and, when `parent` is
// set, the sensor's place as static TF <parent> -> <lidar_name>_link (`xyz` m, `rpy` rad). An
// upside-down sensor is a roll of pi there. Packets from anyone but `sensor_ip` (when set) are
// ignored.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rby1_tools/camera_model.hpp"
#include "rby1_tools/lidar_model.hpp"
#include "rby1_tools/lidar_node.hpp"

class LidarPublisher : public rclcpp::Node {
public:
  LidarPublisher() : rclcpp::Node("lidar_publisher") {
    const auto lidar_name = declare_parameter("lidar_name", std::string(get_name()));
    const auto host_ip = declare_parameter("host_ip", "0.0.0.0");
    port_ = static_cast<int>(declare_parameter("port", 0));  // no default: two sensors must not share one
    sensor_ip_ = declare_parameter("sensor_ip", "");
    const auto parent = declare_parameter("parent", "");
    const auto xyz = declare_parameter("xyz", std::vector<double>{0.0, 0.0, 0.0});
    const auto rpy = declare_parameter("rpy", std::vector<double>{0.0, 0.0, 0.0});
    range_min_ = declare_parameter("range_min", 0.0);
    range_max_ = declare_parameter("range_max", 100.0);
    const bool merge = declare_parameter("merge", false);  // the scan to lidar_merger only
    if (xyz.size() != 3 || rpy.size() != 3) throw std::invalid_argument("xyz and rpy need 3 values each");
    if (port_ == 0) {
      throw std::invalid_argument("no port for " + std::string(get_name()) + ": give it its section in the settings "
                                  "(config/lidar/lidar.yaml) with the sensor's host port");
    }
    if (port_ < 0 || port_ > 65535) throw std::invalid_argument("port must be 1..65535");
    frame_ = rby1_tools::frame_name(lidar_name, "link");
    const auto topic = rby1_tools::lidar_scan_topic(lidar_name, merge);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);

    if (!parent.empty()) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = parent;
      t.child_frame_id = frame_;
      t.transform.translation.x = xyz[0];
      t.transform.translation.y = xyz[1];
      t.transform.translation.z = xyz[2];
      tf2::Quaternion q;
      q.setRPY(rpy[0], rpy[1], rpy[2]);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
      tf_->sendTransform(t);
    }

    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) throw std::runtime_error(std::string("cannot open a UDP socket: ") + std::strerror(errno));
    timeval timeout{1, 0};  // recvfrom wakes up every second, to notice silence and shutdown
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_ip.c_str(), &address.sin_addr) != 1) {
      throw std::invalid_argument("host_ip '" + host_ip + "' is not an IPv4 address");
    }
    if (bind(socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error("cannot listen on " + host_ip + ":" + std::to_string(port_) + ": " +
                               std::strerror(errno) + " (another program on that port? ss -lunp)");
    }
    RCLCPP_INFO(get_logger(), "listening on %s:%d%s; publishing %s in %s%s", host_ip.c_str(), port_,
                sensor_ip_.empty() ? "" : (" for " + sensor_ip_).c_str(), topic.c_str(), frame_.c_str(),
                parent.empty() ? "" : (" (on " + parent + ")").c_str());
    running_ = true;
    worker_ = std::thread([this] { loop(); });
  }

  ~LidarPublisher() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    if (socket_ >= 0) close(socket_);
  }

private:
  void loop() {
    std::vector<uint8_t> buffer(2048);
    rby1_tools::ScanAssembler assembler;
    uint16_t resolution = 0;
    auto last_packet = std::chrono::steady_clock::now();
    rclcpp::Time turn_start = now();
    while (running_ && rclcpp::ok()) {
      sockaddr_in from{};
      socklen_t from_size = sizeof(from);
      const auto size = recvfrom(socket_, buffer.data(), buffer.size(), 0, reinterpret_cast<sockaddr *>(&from),
                                 &from_size);
      if (size < 0) {
        if (std::chrono::steady_clock::now() - last_packet > std::chrono::seconds(2)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "no packets on port %d%s: is the sensor on, and is this PC its host (curl "
                               "http://<sensor>/api/v1/sensor/overview shows its \"host\")?",
                               port_, sensor_ip_.empty() ? "" : (" from " + sensor_ip_).c_str());
        }
        continue;
      }
      char from_ip[INET_ADDRSTRLEN] = {};
      inet_ntop(AF_INET, &from.sin_addr, from_ip, sizeof(from_ip));
      if (!sensor_ip_.empty() && sensor_ip_ != from_ip) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring packets from %s on port %d (sensor_ip %s)",
                             from_ip, port_, sensor_ip_.c_str());
        continue;
      }
      last_packet = std::chrono::steady_clock::now();
      const auto returns = rby1_tools::parse_lakibeam(buffer.data(), static_cast<size_t>(size), resolution);
      if (returns.empty()) continue;
      const auto stamp = now();
      const bool was_started = assembler.started();
      const bool finished = assembler.add(returns, resolution);
      if (!was_started && assembler.started()) turn_start = stamp;  // the first whole turn begins
      if (!finished) continue;
      publish(assembler, turn_start, (stamp - turn_start).seconds());  // stamped when the turn began
      turn_start = stamp;
    }
  }

  void publish(const rby1_tools::ScanAssembler & assembler, const rclcpp::Time & stamp, double scan_time) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame_;
    const auto & ranges = assembler.ranges();
    scan.angle_increment = static_cast<float>(assembler.resolution() * M_PI / 18000.0);
    scan.angle_min = static_cast<float>(-M_PI);
    scan.angle_max = scan.angle_min + scan.angle_increment * static_cast<float>(ranges.size() - 1);
    scan.scan_time = static_cast<float>(scan_time);
    scan.time_increment = ranges.empty() ? 0.0f : static_cast<float>(scan_time / ranges.size());
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = ranges;
    scan.intensities = assembler.intensities();
    scan_pub_->publish(scan);
  }

  int socket_ = -1;
  int port_ = 2368;
  std::string sensor_ip_, frame_;
  double range_min_ = 0.0, range_max_ = 100.0;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<LidarPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_publisher"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
