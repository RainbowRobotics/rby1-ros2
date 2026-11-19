// LakiBeam1 packets and the turns they add up to, from packets built here, and scans put together
// or matched, from a floor plan cast here; no ROS graph, no sensor.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "rby1_tools/lidar_model.hpp"

using namespace rby1_tools;

namespace {

void put(std::vector<uint8_t> & p, size_t at, uint16_t value) {
  p[at] = static_cast<uint8_t>(value & 0xFF);
  p[at + 1] = static_cast<uint8_t>(value >> 8);
}

// A data packet whose block b starts at azimuth `first` + b * 16 * `resolution` (0.01 deg); each
// return's distance is its azimuth / 10 mm (0 for azimuths in `blank`), intensity 7.
std::vector<uint8_t> packet(uint16_t first, uint16_t resolution, int blank_from = -1, int blank_to = -1) {
  std::vector<uint8_t> p(kLakibeamPacketSize, 0);
  for (size_t b = 0; b < 12; ++b) {
    const size_t at = b * 100;
    const uint16_t azimuth = static_cast<uint16_t>((first + b * 16 * resolution) % 36000);
    put(p, at, 0xEEFF);
    put(p, at + 2, azimuth);
    for (size_t i = 0; i < 16; ++i) {
      const int a = (azimuth + resolution * i) % 36000;
      const bool blank = a >= blank_from && a < blank_to;
      put(p, at + 4 + i * 6, blank ? 0 : static_cast<uint16_t>(a / 10));
      p[at + 4 + i * 6 + 2] = 7;
    }
  }
  return p;
}

}  // namespace

TEST(Lakibeam, PacketGivesItsReturns) {
  uint16_t resolution = 0;
  const auto p = packet(9000, 25);
  const auto returns = parse_lakibeam(p.data(), p.size(), resolution);
  EXPECT_EQ(resolution, 25);  // 0.25 deg, from two blocks 16 returns apart
  ASSERT_EQ(returns.size(), 12u * 16u);
  EXPECT_EQ(returns[0].azimuth, 9000);
  EXPECT_EQ(returns[1].azimuth, 9025);
  EXPECT_EQ(returns[16].azimuth, 9400);  // the second block
  EXPECT_EQ(returns[0].distance, 900);
  EXPECT_EQ(returns[0].intensity, 7);
}

TEST(Lakibeam, AnythingElseIsNotAPacket) {
  uint16_t resolution = 0;
  std::vector<uint8_t> short_one(100, 0);
  EXPECT_TRUE(parse_lakibeam(short_one.data(), short_one.size(), resolution).empty());
  std::vector<uint8_t> no_flag(kLakibeamPacketSize, 0);
  EXPECT_TRUE(parse_lakibeam(no_flag.data(), no_flag.size(), resolution).empty());
  EXPECT_EQ(resolution, 0);
}

TEST(Lakibeam, AzimuthWrapsAtAFullTurn) {
  uint16_t resolution = 0;
  const auto p = packet(35600, 25);  // its second block goes past 360 deg
  const auto returns = parse_lakibeam(p.data(), p.size(), resolution);
  EXPECT_EQ(resolution, 25);  // read across 0 too
  ASSERT_EQ(returns.size(), 12u * 16u);
  EXPECT_EQ(returns[15].azimuth, 35975);
  EXPECT_EQ(returns[16].azimuth, 0);
}

TEST(Assembler, AWholeTurnIsPublishedOnceAndThePartFirstOneIsNot) {
  ScanAssembler assembler;
  uint16_t resolution = 0;
  int finished = 0;
  const uint16_t per_packet = 12 * 16 * 25;  // 48 deg
  // Start mid-turn at 180 deg, then run two and a half turns; azimuths 45..60 deg have no return.
  for (int start = 18000; start < 18000 + 36000 * 2 + 18000; start += per_packet) {
    const auto p = packet(static_cast<uint16_t>(start % 36000), 25, 4500, 6000);
    if (assembler.add(parse_lakibeam(p.data(), p.size(), resolution), resolution)) ++finished;
  }
  EXPECT_EQ(finished, 2);  // the turns from 0 deg; the first, part one is dropped
  const auto & ranges = assembler.ranges();
  ASSERT_EQ(ranges.size(), 1440u);  // 360 / 0.25
  EXPECT_EQ(assembler.resolution(), 25);
  EXPECT_TRUE(std::isinf(ranges[0]));  // azimuth 0: distance 0 mm, no return
  EXPECT_FLOAT_EQ(ranges[400], 1.0f);  // azimuth 100 deg: 1000 mm
  EXPECT_TRUE(std::isinf(ranges[200]));  // 50 deg, in the blank part
  EXPECT_FLOAT_EQ(assembler.intensities()[400], 7.0f);
}

namespace {

struct Segment {
  double x0, y0, x1, y1;
};

// A room 8 x 5.5 m with a box and a pillar in it, seen by a level LakiBeam1 (270 deg, 0.25 deg)
// at `pose` on the floor plan; every range off by up to `noise` m (a fixed pseudo-random sequence).
const std::vector<Segment> kRoom{
    {-3.0, -2.5, 5.0, -2.5}, {5.0, -2.5, 5.0, 3.0}, {5.0, 3.0, -3.0, 3.0}, {-3.0, 3.0, -3.0, -2.5},
    {1.5, 0.5, 2.0, 0.5},    {2.0, 0.5, 2.0, 1.2},  {2.0, 1.2, 1.5, 1.2},  {1.5, 1.2, 1.5, 0.5},
    {3.0, -1.2, 3.3, -1.2},  {3.3, -1.2, 3.3, -0.9}, {3.3, -0.9, 3.0, -0.9}, {3.0, -0.9, 3.0, -1.2}};

ScanData scan_of(const std::vector<Segment> & world, const Pose2 & pose, double noise, double range_max = 30.0) {
  ScanData scan;
  scan.angle_min = static_cast<float>(-M_PI);
  scan.angle_increment = static_cast<float>(2.0 * M_PI / 1440.0);
  scan.range_max = static_cast<float>(range_max);
  scan.ranges.assign(1440, std::numeric_limits<float>::infinity());
  uint32_t seed = 12345;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    seed = seed * 1664525u + 1013904223u;
    const double angle = scan.angle_min + i * static_cast<double>(scan.angle_increment);
    if (std::abs(angle) > 135.0 * M_PI / 180.0) continue;  // behind the sensor
    const double dx = std::cos(pose.yaw + angle), dy = std::sin(pose.yaw + angle);
    double best = range_max;
    for (const auto & w : world) {
      const double ex = w.x1 - w.x0, ey = w.y1 - w.y0;
      const double det = ex * dy - ey * dx;
      if (std::abs(det) < 1e-12) continue;
      const double t = (ex * (w.y0 - pose.y) - ey * (w.x0 - pose.x)) / det;     // along the beam
      const double u = (dx * (w.y0 - pose.y) - dy * (w.x0 - pose.x)) / det;     // along the wall
      if (t > 0.0 && t < best && u >= 0.0 && u <= 1.0) best = t;
    }
    if (best < range_max) scan.ranges[i] = static_cast<float>(best + noise * ((seed >> 8) / 8388608.0 - 1.0));
  }
  return scan;
}

Pose3 level(const Pose2 & p) { return pose_from_xyz_rpy({p.x, p.y, 0.22}, {0.0, 0.0, p.yaw}); }

Pose2 moved(const Pose2 & p, const Pose2 & by) {
  const double c = std::cos(by.yaw), s = std::sin(by.yaw);
  return {c * p.x - s * p.y + by.x, s * p.x + c * p.y + by.y, p.yaw + by.yaw};
}

}  // namespace

TEST(Pose, RpyRoundTripsAsTf) {
  const auto rpy = rpy_of(pose_from_xyz_rpy({0.2, -0.1, 0.3}, {0.1, -0.2, 0.959931}));
  EXPECT_NEAR(rpy[0], 0.1, 1e-12);
  EXPECT_NEAR(rpy[1], -0.2, 1e-12);
  EXPECT_NEAR(rpy[2], 0.959931, 1e-12);
  const auto flipped = rpy_of(pose_from_xyz_rpy({0.0, 0.0, 0.0}, {M_PI, 0.0, -0.5}));  // upside down
  EXPECT_NEAR(std::abs(flipped[0]), M_PI, 1e-12);
  EXPECT_NEAR(flipped[2], -0.5, 1e-12);
}

TEST(ScanPoints, PlacesEachReturnByThePose) {
  ScanData scan;
  scan.angle_min = 0.0f;
  scan.angle_increment = static_cast<float>(M_PI / 2.0);
  scan.range_min = 0.1f;
  scan.range_max = 10.0f;
  scan.ranges = {2.0f, std::numeric_limits<float>::infinity(), 0.05f, 20.0f};  // only the first counts
  scan.intensities = {9.0f, 0.0f, 0.0f, 0.0f};
  const auto points = scan_points(scan, pose_from_xyz_rpy({1.0, 0.0, 0.5}, {0.0, 0.0, M_PI / 2.0}));
  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points[0].x, 1.0, 1e-9);  // 2 m ahead of a sensor turned to +y
  EXPECT_NEAR(points[0].y, 2.0, 1e-9);
  EXPECT_FLOAT_EQ(points[0].intensity, 9.0f);

  scan.ranges = {std::numeric_limits<float>::infinity(), 2.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f};
  const auto flipped = scan_points(scan, pose_from_xyz_rpy({0.0, 0.0, 0.0}, {M_PI, 0.0, 0.0}));
  ASSERT_EQ(flipped.size(), 1u);
  EXPECT_NEAR(flipped[0].y, -2.0, 1e-9);  // its left is the frame's right
}

TEST(Merge, EachSectorKeepsItsNearestFromTheOrigin) {
  std::vector<Point2> points{{2.0, 0.0, 1.0f}, {1.0, 0.0, 2.0f}, {-1.0, 0.0, 3.0f}, {0.0, 0.05, 4.0f},
                             {0.0, 50.0, 5.0f}};
  std::vector<float> ranges, intensities;
  merge_points(points, 4, 0.1, 10.0, ranges, intensities);  // sectors from -pi: [-pi,-pi/2) .. [pi/2,pi)
  ASSERT_EQ(ranges.size(), 4u);
  EXPECT_FLOAT_EQ(ranges[2], 1.0f);  // angle 0: the nearer of 2 m and 1 m
  EXPECT_FLOAT_EQ(intensities[2], 2.0f);
  EXPECT_FLOAT_EQ(ranges[0], 1.0f);  // angle +pi is -pi: sector 0, not one past the end
  EXPECT_TRUE(std::isinf(ranges[1]));
  EXPECT_TRUE(std::isinf(ranges[3]));  // 5 cm is under range_min, 50 m over range_max
}

TEST(Align, FindsHowFarOffASensorIsPlaced) {
  const Pose2 left{0.2525, 0.1765, 0.959931}, right{0.2525, -0.1765, -0.959931};
  const Pose2 believed = moved(right, {0.03, -0.02, 1.5 * M_PI / 180.0});  // the settings are off
  const auto reference = scan_points(scan_of(kRoom, left, 0.003), level(left));
  const auto moving = scan_points(scan_of(kRoom, right, 0.003), level(believed));
  const auto a = align_points(reference, moving);
  ASSERT_TRUE(a.converged);
  EXPECT_FALSE(a.degenerate);
  EXPECT_GE(a.matches, kMinMatches);
  const Pose2 found = moved(believed, a.correction);
  EXPECT_NEAR(found.x, right.x, 0.003);
  EXPECT_NEAR(found.y, right.y, 0.003);
  EXPECT_NEAR(found.yaw, right.yaw, 0.1 * M_PI / 180.0);
  EXPECT_LT(a.rms, 0.005);
}

TEST(Align, ACorridorDoesNotPinItsLength) {
  const std::vector<Segment> corridor{{-100.0, -1.0, 100.0, -1.0}, {-100.0, 1.0, 100.0, 1.0}};
  const Pose2 left{0.2525, 0.1765, 0.959931}, right{0.2525, -0.1765, -0.959931};
  const auto reference = scan_points(scan_of(corridor, left, 0.003, 15.0), level(left));
  const auto moving = scan_points(scan_of(corridor, right, 0.003, 15.0), level(moved(right, {0.05, 0.0, 0.0})));
  EXPECT_TRUE(align_points(reference, moving).degenerate);
}
