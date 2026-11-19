// realsense_publisher: an Intel RealSense (librealsense, no realsense-ros) as ROS topics,
// only the streams asked for. Settings: config/camera/realsense.yaml.
//
//   ros2 launch rby1_tools camera.launch.py [camera:=realsense] [camera_name:=camera] [mount:=head_default]
//
// use_rgb / use_depth / use_ir / use_point_cloud pick what is published; a stream the
// device does not offer at width x height @ fps is warned about and left out. Each image
// stream (rgb, depth, ir_left, ir_right) goes out on /<camera_name>/<stream>/raw -- with
// `compressed`, on /<camera_name>/<stream>/compressed instead, JPEG (depth: PNG, lossless)
// -- with its CameraInfo on /<camera_name>/<stream>/info: colour as bgr8, depth as 16UC1 in
// millimetres, infrared as mono8. The depth as points (x, y, z in m) goes out on
// /<camera_name>/point_cloud. The colour camera model is the device's factory calibration,
// or `intrinsics_file` when one is named (a calibration in config/camera/intrinsics/).
//
// Frames, as static TF: the mount (`mount` in camera_mount.yaml) -> <camera_name>_link ->
// <camera_name>_optical_frame (colour), and under it, by the device's extrinsics,
// <camera_name>_depth_optical_frame (depth, left infrared) and
// <camera_name>_ir_right_optical_frame -- or depth is aligned to colour (align_depth_to_color).
//
// As camera_ws does: the first frames are dropped while exposure settles, and auto
// exposure may not lower the frame rate (auto_exposure_priority off).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rby1_tools/camera_model.hpp"
#include "rby1_tools/camera_config.hpp"
#include "rby1_tools/image_publisher.hpp"

namespace {

sensor_msgs::msg::CameraInfo info_from(const rs2_intrinsics & in, const std::string & frame_id) {
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = static_cast<uint32_t>(in.width);
  info.height = static_cast<uint32_t>(in.height);
  info.distortion_model = "plumb_bob";  // Brown-Conrady; D4xx colour reports near-zero coefficients
  info.d = {in.coeffs[0], in.coeffs[1], in.coeffs[2], in.coeffs[3], in.coeffs[4]};
  info.k = {in.fx, 0.0, in.ppx, 0.0, in.fy, in.ppy, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {in.fx, 0.0, in.ppx, 0.0, 0.0, in.fy, in.ppy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

bool has_stream(const rs2::sensor & sensor, rs2_stream stream) {
  for (const auto & profile : sensor.get_stream_profiles()) {
    if (profile.stream_type() == stream) return true;
  }
  return false;
}

// A profile of `stream` (`index`: infrared 1 or 2, -1 any) the device offers, at this size,
// rate and format -- or, with width 0, at any.
std::optional<rs2::stream_profile> offered(const rs2::device & device, rs2_stream stream, int index,
                                           rs2_format format, int width, int height, int fps) {
  for (auto && sensor : device.query_sensors()) {
    for (auto && profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() != stream || (index >= 0 && profile.stream_index() != index)) continue;
      if (width == 0) return profile;
      if (profile.format() != format || profile.fps() != fps || !profile.is<rs2::video_stream_profile>()) continue;
      const auto video = profile.as<rs2::video_stream_profile>();
      if (video.width() == width && video.height() == height) return profile;
    }
  }
  return std::nullopt;
}

}  // namespace

// A RealSense that is on USB but has lost the kernel's video driver: what to do about it, or "".
// A program that opens the camera through libusb takes it from the kernel driver, and when it
// ends without handing it back, nothing that reads cameras the usual way (this node, the
// rs-* tools) sees the camera until it is plugged in again.
std::string detached_camera() {
  namespace fs = std::filesystem;
  std::error_code ignored;
  for (const auto & entry : fs::directory_iterator("/sys/bus/usb/devices", ignored)) {
    std::string vendor, product;
    std::ifstream(entry.path() / "idVendor") >> vendor;
    std::getline(std::ifstream(entry.path() / "product"), product);
    if (vendor != "8086" || product.find("RealSense") == std::string::npos) continue;
    const auto first_interface = entry.path().string() + ":1.0";
    if (fs::exists(first_interface, ignored) && !fs::exists(first_interface + "/driver", ignored)) {
      return "the " + product + " is on USB (" + entry.path().filename().string() + ") but the kernel's video "
             "driver is not attached to it: another program took the camera (through libusb) and did not give it "
             "back. Close that program, then unplug the camera and plug it in again (or: sudo modprobe -r uvcvideo "
             "&& sudo modprobe uvcvideo)";
    }
  }
  return "";
}

class RealSensePublisher : public rclcpp::Node {
public:
  RealSensePublisher() : rclcpp::Node("realsense_publisher") {
    const auto serial = declare_parameter("serial", "");
    const int width = static_cast<int>(declare_parameter("width", 1280));
    const int height = static_cast<int>(declare_parameter("height", 720));
    const int fps = static_cast<int>(declare_parameter("fps", 30));
    const bool use_rgb = declare_parameter("use_rgb", true);
    const bool use_depth = declare_parameter("use_depth", false);
    const bool use_ir = declare_parameter("use_ir", false);
    const bool use_point_cloud = declare_parameter("use_point_cloud", false);
    const bool align = declare_parameter("align_depth_to_color", false);
    const auto camera_name = declare_parameter("camera_name", "camera");
    const auto mount = declare_parameter("mount", "head_default");
    const bool auto_exposure = declare_parameter("auto_exposure", true);
    const double exposure = declare_parameter("exposure", 6000.0);  // us
    const bool ir_auto_exposure = declare_parameter("ir_auto_exposure", true);
    const double ir_exposure = declare_parameter("ir_exposure", 8500.0);  // us
    const auto intrinsics_file = rby1_tools::intrinsics_path(declare_parameter("intrinsics_file", ""));
    skip_frames_ = static_cast<int>(declare_parameter("skip_frames", 10));
    const bool compressed = declare_parameter("compressed", false);
    if (!use_rgb && !use_depth && !use_ir && !use_point_cloud) {
      throw std::runtime_error("nothing on: set use_rgb, use_depth, use_ir or use_point_cloud");
    }
    const auto color_frame = rby1_tools::frame_name(camera_name, "optical_frame");
    const auto depth_frame = rby1_tools::frame_name(camera_name, "depth_optical_frame");
    const auto ir_right_frame = rby1_tools::frame_name(camera_name, "ir_right_optical_frame");

    // The device: the one with `serial`, or the only/first one.
    rs2::context context;
    auto devices = context.query_devices();
    if (devices.size() == 0) {
      throw std::runtime_error(detached_camera().empty()
                                 ? "no RealSense found -- is it plugged into a USB 3 port (rs-enumerate-devices)?"
                                 : detached_camera());
    }
    std::optional<rs2::device> device;
    std::string connected;
    for (auto && candidate : devices) {
      connected += (connected.empty() ? "" : ", ") + std::string(candidate.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) +
                   " (" + candidate.get_info(RS2_CAMERA_INFO_NAME) + ")";
      if (!device && (serial.empty() || serial == candidate.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))) {
        device = candidate;
      }
    }
    if (!device) throw std::runtime_error("no RealSense with serial " + serial + " -- connected: " + connected);
    const std::string name = device->get_info(RS2_CAMERA_INFO_NAME);
    const std::string found_serial = device->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    const std::string usb = device->supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                              ? device->get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) : "?";
    RCLCPP_INFO(get_logger(), "%s (serial %s, USB %s)", name.c_str(), found_serial.c_str(), usb.c_str());
    if (!usb.empty() && usb[0] == '2') {
      RCLCPP_WARN(get_logger(), "on a USB 2 link: high resolutions and frame rates will not open -- use a USB 3 "
                                "cable and port");
    }

    // What the device offers at this mode; whatever it does not is left out, with a warning.
    const std::string mode = std::to_string(width) + "x" + std::to_string(height) + " @ " + std::to_string(fps) + " fps";
    auto check = [&](bool wanted, rs2_stream stream, int index, rs2_format format, const char * what,
                     const char * flag) {
      if (!wanted) return false;
      if (offered(*device, stream, index, format, width, height, fps)) return true;
      RCLCPP_WARN(get_logger(), "%s: the %s has no %s stream at %s -- it is not published (rs-enumerate-devices shows "
                                "the modes)", flag, name.c_str(), what, mode.c_str());
      return false;
    };
    const bool rgb_on = check(use_rgb, RS2_STREAM_COLOR, -1, RS2_FORMAT_BGR8, "colour (bgr8)", "use_rgb");
    const bool depth_on = check(use_depth || use_point_cloud, RS2_STREAM_DEPTH, -1, RS2_FORMAT_Z16, "depth (z16)",
                                use_depth ? "use_depth" : "use_point_cloud");
    const bool ir_left_on = check(use_ir, RS2_STREAM_INFRARED, 1, RS2_FORMAT_Y8, "left infrared (y8)", "use_ir");
    const bool ir_right_on = check(use_ir, RS2_STREAM_INFRARED, 2, RS2_FORMAT_Y8, "right infrared (y8)", "use_ir");
    if (!rgb_on && !depth_on && !ir_left_on && !ir_right_on) {
      throw std::runtime_error("the " + name + " offers none of the streams asked for at " + mode +
                               " (rs-enumerate-devices shows the modes)");
    }
    align_ = align && rgb_on && depth_on;
    if (align && !align_) {
      RCLCPP_WARN(get_logger(), "align_depth_to_color needs colour and depth: depth is not aligned");
    }

    rs2::config config;
    config.enable_device(found_serial);
    if (rgb_on) config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    if (depth_on) config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    if (ir_left_on) config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    if (ir_right_on) config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    try {
      profile_ = pipeline_.start(config);
    } catch (const rs2::error & error) {
      throw std::runtime_error("the RealSense could not open " + mode + " with these streams together (" +
                               error.what() + ") -- fewer streams, a lower mode, or USB 3");
    }

    // Exposure: colour on the sensor that carries colour (the depth module on a D405),
    // infrared on the stereo module.
    auto sensors = profile_.get_device().query_sensors();
    std::optional<rs2::sensor> color_sensor, stereo_sensor;
    for (auto && sensor : sensors) {
      if (has_stream(sensor, RS2_STREAM_COLOR) && !color_sensor) color_sensor = sensor;
      if (has_stream(sensor, RS2_STREAM_DEPTH) && !stereo_sensor) stereo_sensor = sensor;
    }
    auto set = [this](rs2::sensor & sensor, rs2_option option, float value, const char * what) {
      if (!sensor.supports(option)) return;
      try {
        sensor.set_option(option, value);
      } catch (const rs2::error & error) {
        RCLCPP_WARN(get_logger(), "could not set %s to %.0f: %s", what, value, error.what());
      }
    };
    if (rgb_on && color_sensor) {
      set(*color_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure ? 1.f : 0.f, "auto exposure");
      if (!auto_exposure) set(*color_sensor, RS2_OPTION_EXPOSURE, static_cast<float>(exposure), "exposure");
      if (color_sensor->supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
        set(*color_sensor, RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f, "auto_exposure_priority");  // hold the fps
      } else if (auto_exposure) {
        RCLCPP_WARN(get_logger(), "%s cannot hold the frame rate under auto exposure (no auto_exposure_priority): in "
                                  "dim light it drops -- set auto_exposure false and an exposure if it matters",
                    name.c_str());
      }
    }
    const bool stereo_on = depth_on || ir_left_on || ir_right_on;
    if (stereo_sensor) {
      const bool shared = color_sensor && color_sensor->get_info(RS2_CAMERA_INFO_NAME) ==
                                            stereo_sensor->get_info(RS2_CAMERA_INFO_NAME);
      if (stereo_on && !(shared && rgb_on)) {
        set(*stereo_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, ir_auto_exposure ? 1.f : 0.f, "infrared auto exposure");
        if (!ir_auto_exposure) set(*stereo_sensor, RS2_OPTION_EXPOSURE, static_cast<float>(ir_exposure), "infrared exposure");
      } else if (shared && rgb_on && stereo_on) {
        RCLCPP_INFO(get_logger(), "%s: colour and infrared share one sensor -- the colour exposure settings apply",
                    name.c_str());
      }
      depth_scale_ = profile_.get_device().first<rs2::depth_sensor>().get_depth_scale();
    }

    // Frames: the mount and the colour frame, and the depth and infrared ones under it by the
    // device's extrinsics -- taken from a colour profile even when colour is not streamed.
    auto profile_of = [this](rs2_stream stream, int index) {
      return profile_.get_stream(stream, index).as<rs2::video_stream_profile>();
    };
    auto transforms = rby1_tools::mount_transforms(this, camera_name, mount);
    const auto color_profile = rgb_on ? std::optional<rs2::stream_profile>(profile_of(RS2_STREAM_COLOR, -1))
                                      : offered(*device, RS2_STREAM_COLOR, -1, RS2_FORMAT_ANY, 0, 0, 0);
    if (!color_profile && stereo_on) {
      RCLCPP_WARN(get_logger(), "the %s has no colour camera: the depth and infrared frames are not tied to %s",
                  name.c_str(), color_frame.c_str());
    }
    auto under_color = [&](const rs2::stream_profile & from, const std::string & child) {
      if (!color_profile) return;
      const auto e = from.get_extrinsics_to(*color_profile);  // from -> colour
      const Eigen::Matrix3f rotation = Eigen::Map<const Eigen::Matrix3f>(e.rotation);  // column-major
      const Eigen::Quaternionf q(rotation);
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now();
      t.header.frame_id = color_frame;
      t.child_frame_id = child;
      t.transform.translation.x = e.translation[0];
      t.transform.translation.y = e.translation[1];
      t.transform.translation.z = e.translation[2];
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      transforms.push_back(t);
    };
    // Reliable: Isaac ROS subscribes reliably; a reliable publisher serves best-effort ones too.
    std::vector<std::string> published;
    auto make = [&](const std::string & stream_name, const std::string & format = "jpeg") {
      const auto topic = rby1_tools::stream_topic(camera_name, stream_name);
      Stream stream;
      stream.image = rby1_tools::image_publisher(this, topic, compressed, format);
      stream.info = create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/info", 5);
      published.push_back(rby1_tools::image_topic(topic, compressed));
      return stream;
    };
    if (rgb_on) {
      rgb_ = make("rgb");
      if (!intrinsics_file.empty()) {
        rgb_->info_msg = rby1_tools::camera_info(rby1_tools::read_intrinsics(intrinsics_file), width, height,
                                                 color_frame);
      } else {
        rgb_->info_msg = info_from(profile_of(RS2_STREAM_COLOR, -1).get_intrinsics(), color_frame);
      }
      RCLCPP_INFO(get_logger(), "colour model: %s (fx %.1f fy %.1f)",
                  intrinsics_file.empty() ? "factory calibration" : intrinsics_file.c_str(), rgb_->info_msg.k[0],
                  rgb_->info_msg.k[4]);
    }
    if (depth_on && !align_) under_color(profile_of(RS2_STREAM_DEPTH, -1), depth_frame);
    if (depth_on && use_depth) {
      depth_ = make("depth", "png");
      depth_->info_msg = align_ ? rgb_->info_msg  // aligned: colour pixels, colour frame
                                : info_from(profile_of(RS2_STREAM_DEPTH, -1).get_intrinsics(), depth_frame);
    }
    if (ir_left_on) {
      ir_left_ = make("ir_left");
      ir_left_->info_msg = info_from(profile_of(RS2_STREAM_INFRARED, 1).get_intrinsics(), depth_frame);
      if (!depth_on || align_) under_color(profile_of(RS2_STREAM_INFRARED, 1), depth_frame);  // IR 1 is depth's frame
    }
    if (ir_right_on) {
      ir_right_ = make("ir_right");
      ir_right_->info_msg = info_from(profile_of(RS2_STREAM_INFRARED, 2).get_intrinsics(), ir_right_frame);
      under_color(profile_of(RS2_STREAM_INFRARED, 2), ir_right_frame);
    }
    if (depth_on && use_point_cloud) {
      cloud_frame_ = align_ ? color_frame : depth_frame;
      cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(rby1_tools::point_cloud_topic(camera_name), 5);
      published.push_back(cloud_pub_->get_topic_name());
    }
    tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_->sendTransform(transforms);
    if (align_) aligner_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

    std::string topics;
    for (const auto & topic : published) topics += " " + topic;
    RCLCPP_INFO(get_logger(), "streaming %s%s:%s (mount: %s)", mode.c_str(), align_ ? ", depth aligned to colour" : "",
                topics.c_str(), rby1_tools::mounted(mount) ? mount.c_str() : "none");
    fps_ = fps;
    running_ = true;
    worker_ = std::thread([this] { loop(); });
  }

  ~RealSensePublisher() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    try {
      pipeline_.stop();
    } catch (...) {
    }
  }

private:
  struct Stream {
    image_transport::Publisher image;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
    sensor_msgs::msg::CameraInfo info_msg;
  };

  void publish(Stream & stream, const rs2::video_frame & frame, const std::string & encoding, int bytes_per_pixel,
               const std_msgs::msg::Header & header) {
    sensor_msgs::msg::Image image;
    image.header = header;
    image.header.frame_id = stream.info_msg.header.frame_id;
    image.width = static_cast<uint32_t>(frame.get_width());
    image.height = static_cast<uint32_t>(frame.get_height());
    image.encoding = encoding;
    image.step = static_cast<uint32_t>(frame.get_width() * bytes_per_pixel);
    const auto * data = static_cast<const uint8_t *>(frame.get_data());
    image.data.assign(data, data + image.step * image.height);
    if (encoding == "16UC1" && std::abs(depth_scale_ - 0.001f) > 1e-7f) {
      // Depth in millimetres whatever the device's unit (a D405 counts 0.1 mm).
      auto * values = reinterpret_cast<uint16_t *>(image.data.data());
      const float to_mm = depth_scale_ * 1000.0f;
      for (size_t i = 0; i < image.data.size() / 2; ++i) {
        values[i] = static_cast<uint16_t>(std::min(65535.0f, std::round(values[i] * to_mm)));
      }
    }
    stream.image.publish(image);
    stream.info_msg.header.stamp = header.stamp;
    stream.info->publish(stream.info_msg);
  }

  // The depth image as points in metres (x, y, z), one per pixel in image order; pixels
  // without depth are NaN.
  void publish_cloud(const rs2::depth_frame & depth, const std_msgs::msg::Header & header) {
    const rs2::points points = pointcloud_.calculate(depth);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = cloud_frame_;
    cloud.height = static_cast<uint32_t>(depth.get_height());
    cloud.width = static_cast<uint32_t>(depth.get_width());
    for (const char * axis : {"x", "y", "z"}) {
      sensor_msgs::msg::PointField field;
      field.name = axis;
      field.offset = static_cast<uint32_t>(cloud.fields.size() * sizeof(float));
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.is_bigendian = false;
    cloud.point_step = 3 * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = false;
    cloud.data.resize(static_cast<size_t>(cloud.row_step) * cloud.height);
    auto * out = reinterpret_cast<float *>(cloud.data.data());
    const auto * vertices = points.get_vertices();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const size_t count = std::min(points.size(), static_cast<size_t>(cloud.width) * cloud.height);
    for (size_t i = 0; i < count; ++i) {
      const bool valid = vertices[i].z > 0.0f;
      out[3 * i] = valid ? vertices[i].x : nan;
      out[3 * i + 1] = valid ? vertices[i].y : nan;
      out[3 * i + 2] = valid ? vertices[i].z : nan;
    }
    cloud_pub_->publish(cloud);
  }

  // Every 5 s: is the stream as fast as asked? If not, say whether the camera delivers the
  // frames late (and at what exposure) or publishing them is what takes the time.
  struct Health {
    std::chrono::steady_clock::time_point since = std::chrono::steady_clock::now();
    int frames = 0;
    double last_stamp = -1.0, longest_gap = 0.0;  // device time, ms
    double publish_total = 0.0, publish_longest = 0.0, exposure = 0.0;  // s, s, us
  };

  void report(Health & health) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - health.since).count();
    if (elapsed < 5.0) return;
    const double rate = health.frames / elapsed;
    const double publish = health.frames ? health.publish_total / health.frames : 0.0;
    if (health.frames > 0 && rate < 0.9 * fps_) {
      if (publish > 0.5 / fps_) {
        RCLCPP_WARN(get_logger(), "%.1f of %d fps: publishing takes %.0f ms a frame (longest %.0f ms) -- the "
                                  "subscribers are slow to take the images; fewer of them, or a smaller image",
                    rate, fps_, publish * 1e3, health.publish_longest * 1e3);
      } else if (health.exposure > 0.9e6 / fps_) {
        RCLCPP_WARN(get_logger(), "%.1f of %d fps: the camera exposes each frame for %.0f ms (auto exposure, too "
                                  "little light for this rate; longest gap %.0f ms) -- add light, or set "
                                  "auto_exposure false with exposure under %.0f us",
                    rate, fps_, health.exposure / 1e3, health.longest_gap, 1e6 / fps_);
      } else {
        RCLCPP_WARN(get_logger(), "%.1f of %d fps: the camera delivers frames late (longest gap %.0f ms, exposure "
                                  "%.0f ms, publishing %.0f ms a frame) -- the USB cable or port, or another "
                                  "program using the camera?",
                    rate, fps_, health.longest_gap, health.exposure / 1e3, publish * 1e3);
      }
    }
    health = Health{};
  }

  void loop() {
    int skipped = 0;
    Health health;
    while (running_ && rclcpp::ok()) {
      rs2::frameset frames;
      try {
        frames = pipeline_.wait_for_frames(1000);
      } catch (const rs2::error & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "no frames from the RealSense: %s -- unplugged, or opened by another program? (one "
                             "program at a time can use a camera)", error.what());
        health = Health{};
        continue;
      }
      if (skipped < skip_frames_) {  // exposure settles first
        ++skipped;
        continue;
      }
      const auto began = std::chrono::steady_clock::now();
      const double stamp = frames.get_timestamp();
      if (health.last_stamp >= 0.0) health.longest_gap = std::max(health.longest_gap, stamp - health.last_stamp);
      health.last_stamp = stamp;
      if (auto color = frames.get_color_frame()) {
        if (color.supports_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE)) {
          health.exposure = static_cast<double>(color.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
        }
      }
      if (aligner_) frames = aligner_->process(frames);
      std_msgs::msg::Header header;
      header.stamp = now();
      if (rgb_) {
        if (auto color = frames.get_color_frame()) publish(*rgb_, color, "bgr8", 3, header);
      }
      if (depth_ || cloud_pub_) {
        if (auto depth = frames.get_depth_frame()) {
          if (depth_) publish(*depth_, depth, "16UC1", 2, header);
          if (cloud_pub_) publish_cloud(depth, header);
        }
      }
      if (ir_left_) {
        if (auto ir = frames.get_infrared_frame(1)) publish(*ir_left_, ir, "mono8", 1, header);
      }
      if (ir_right_) {
        if (auto ir = frames.get_infrared_frame(2)) publish(*ir_right_, ir, "mono8", 1, header);
      }
      const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      ++health.frames;
      health.publish_total += took;
      health.publish_longest = std::max(health.publish_longest, took);
      report(health);
    }
  }

  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;
  std::unique_ptr<rs2::align> aligner_;
  rs2::pointcloud pointcloud_;
  bool align_ = false;
  float depth_scale_ = 0.001f;
  int skip_frames_ = 10;
  int fps_ = 30;
  std::string cloud_frame_;
  std::optional<Stream> rgb_, depth_, ir_left_, ir_right_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<RealSensePublisher>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("realsense_publisher"), "%s", error.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
