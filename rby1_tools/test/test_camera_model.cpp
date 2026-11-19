// Camera calibrations, names and mounts; no ROS graph, no camera.
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "rby1_tools/camera_model.hpp"

using namespace rby1_tools;

namespace {

// `content` in a file of that name in the system's temp folder; its path.
std::string write_file(const std::string & name, const std::string & content) {
  const auto path = (std::filesystem::temp_directory_path() / name).string();
  std::ofstream(path) << content;
  return path;
}

}  // namespace

TEST(Intrinsics, CameraWsLayoutIsRead) {
  // As camera_ws writes it (config/camera_intrinsics_d435.yaml).
  const auto path = write_file("rby1_intrinsics_camera_ws.yaml", R"(camera_matrix:
- - 906.5679246553677
  - 0.0
  - 639.3426898074739
- - 0.0
  - 907.2519543479634
  - 383.9957721342101
- - 0.0
  - 0.0
  - 1.0
device_name: D435
dist_coeffs:
- 0.12183897591222062
- -0.37092493575481705
- 0.0010067941803011953
- 0.001299501989543505
- 0.33076464260693217
height: 720
rms_error: 0.19824150874733465
width: 1280
)");
  const auto intrinsics = read_intrinsics(path);
  EXPECT_EQ(intrinsics.width, 1280);
  EXPECT_NEAR(intrinsics.k[0], 906.5679246553677, 1e-12);
  EXPECT_NEAR(intrinsics.k[5], 383.9957721342101, 1e-12);
  ASSERT_EQ(intrinsics.d.size(), 5u);
  EXPECT_NEAR(intrinsics.d[4], 0.33076464260693217, 1e-12);
  const auto info = camera_info(intrinsics, 1280, 720, "camera_optical_frame");
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  EXPECT_NEAR(info.p[0], info.k[0], 1e-12);
}

TEST(Intrinsics, RosCalibrationLayoutIsRead) {
  const auto path = write_file("rby1_intrinsics_ros.yaml", R"(image_width: 640
image_height: 480
camera_name: webcam
camera_matrix:
  rows: 3
  cols: 3
  data: [600.0, 0.0, 320.0, 0.0, 601.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.05]
)");
  const auto intrinsics = read_intrinsics(path);
  EXPECT_EQ(intrinsics.height, 480);
  EXPECT_NEAR(intrinsics.k[4], 601.0, 1e-12);
  EXPECT_NEAR(intrinsics.d[1], -0.2, 1e-12);
}

TEST(Intrinsics, ScaledToAnotherSizeOfTheSameShape) {
  Intrinsics intrinsics;
  intrinsics.width = 1280;
  intrinsics.height = 720;
  intrinsics.k = {900.0, 0.0, 640.0, 0.0, 902.0, 360.0, 0.0, 0.0, 1.0};
  intrinsics.d = {0.1, 0.0, 0.0, 0.0, 0.0};
  const auto half = camera_info(intrinsics, 640, 360, "c");
  EXPECT_NEAR(half.k[0], 450.0, 1e-12);
  EXPECT_NEAR(half.k[5], 180.0, 1e-12);
  EXPECT_NEAR(half.d[0], 0.1, 1e-12);  // distortion does not scale
  EXPECT_THROW(camera_info(intrinsics, 640, 480, "c"), std::runtime_error);  // 16:9 file, 4:3 image
}

TEST(Intrinsics, BadFilesSayWhy) {
  EXPECT_THROW(read_intrinsics("/nonexistent/intrinsics.yaml"), std::runtime_error);
  const auto path = write_file("rby1_intrinsics_bad.yaml", "width: 1280\nheight: 720\ncamera_matrix: [[1, 0], [0, 1]]\n");
  EXPECT_THROW(read_intrinsics(path), std::runtime_error);
}

TEST(Names, TopicsAndFramesFollowTheCameraName) {
  EXPECT_EQ(stream_topic("camera", "rgb"), "/camera/rgb");
  EXPECT_EQ(stream_topic("wrist_2", "ir_left"), "/wrist_2/ir_left");
  EXPECT_EQ(point_cloud_topic("camera"), "/camera/point_cloud");
  EXPECT_EQ(frame_name("camera", "optical_frame"), "camera_optical_frame");  // the frames before camera_name
  EXPECT_EQ(frame_name("head", "link"), "head_link");
  for (const char * bad : {"", "2cam", "my camera", "cam/era", "cam-1"}) {
    EXPECT_THROW(stream_topic(bad, "rgb"), std::invalid_argument) << bad;
    EXPECT_THROW(frame_name(bad, "link"), std::invalid_argument) << bad;
  }
}

TEST(Mount, NamedEntriesAreRead) {
  const auto path = write_file("rby1_camera_mount.yaml", R"(head:
  parent: link_head_2
  xyz: [0.022, 0.0, 0.04]
  rpy: [0.0, 0.0, 0.0]
wrist:
  parent: ee_right
  xyz: [0.0, 0.05, 0.1]
  rpy: [0.0, 1.5708, 0.0]
)");
  const auto head = read_mount(path, "head");
  EXPECT_EQ(head.parent, "link_head_2");
  EXPECT_NEAR(head.xyz[0], 0.022, 1e-12);
  EXPECT_NEAR(head.xyz[2], 0.04, 1e-12);
  const auto wrist = read_mount(path, "wrist");
  EXPECT_EQ(wrist.parent, "ee_right");
  EXPECT_NEAR(wrist.rpy[1], 1.5708, 1e-12);
}

TEST(Mount, MissingOrBadEntriesSayWhy) {
  const auto path = write_file("rby1_camera_mount_names.yaml",
                               "head:\n  parent: link_head_2\n  xyz: [0, 0, 0]\n  rpy: [0, 0, 0]\n"
                               "short:\n  parent: link_head_2\n  xyz: [0, 0]\n  rpy: [0, 0, 0]\n");
  try {
    read_mount(path, "chest");
    FAIL() << "a missing mount must throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("head, short"), std::string::npos) << error.what();  // lists the names
  }
  EXPECT_THROW(read_mount(path, "short"), std::runtime_error);  // xyz needs 3 values
  EXPECT_THROW(read_mount("/nonexistent/camera_mount.yaml", "head"), std::runtime_error);
}
