// A Richbeam LakiBeam1 2D LiDAR's UDP data packets, the full turns they add up to, and the scans
// of several of them put together on one frame or matched to each other; no ROS.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rby1_tools {

// One return: where (azimuth, 0.01 deg, 0..35999) and what (distance in mm, 0 for none;
// intensity).
struct LidarReturn {
  uint16_t azimuth = 0;
  uint16_t distance = 0;
  uint8_t intensity = 0;
};

// A data packet is 1206 bytes: 12 blocks of {0xFFEE flag, azimuth (0.01 deg), 16 x {distance 1
// (mm), intensity 1, distance 2, intensity 2}}, then a timestamp and a factory word, little
// endian. Each block's 16 returns are `resolution` apart from its azimuth.
inline constexpr size_t kLakibeamPacketSize = 1206;

// The returns of one packet, in order, and its resolution (0.01 deg) in `resolution` when the
// packet shows it (two blocks apart by 16 returns); empty for anything but a data packet.
std::vector<LidarReturn> parse_lakibeam(const uint8_t * data, size_t size, uint16_t & resolution);

// Collects returns into full turns, one bin per `resolution` from azimuth 0. A turn ends when the
// azimuth goes back to the start; the turn the sensor was in when the first returns came is
// dropped (it is not whole).
class ScanAssembler {
public:
  // True when `returns` finished a turn; its bins are then in ranges() and intensities().
  bool add(const std::vector<LidarReturn> & returns, uint16_t resolution);
  const std::vector<float> & ranges() const { return ranges_; }            // m, +inf where no return
  const std::vector<float> & intensities() const { return intensities_; }
  uint16_t resolution() const { return resolution_; }                      // of the finished turn
  bool started() const { return started_; }                                // a whole turn is being built

private:
  void reset(uint16_t resolution);

  bool started_ = false;
  int last_azimuth_ = -1;
  uint16_t resolution_ = 0;
  std::vector<float> building_ranges_, building_intensities_;
  std::vector<float> ranges_, intensities_;
};

// Where a sensor sits in a frame: p_frame = rotation * p_sensor + translation (rotation row major).
struct Pose3 {
  std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
  std::array<double, 3> translation{};
};
Pose3 pose_from_xyz_rpy(const std::array<double, 3> & xyz, const std::array<double, 3> & rpy);  // m, rad (as TF)
std::array<double, 3> rpy_of(const Pose3 & pose);

// A scan as a LaserScan carries it: beam i at angle_min + i * angle_increment in the sensor's
// x-y plane; a beam counts when its range is finite and within [range_min, range_max].
struct ScanData {
  float angle_min = 0.0f, angle_increment = 0.0f, range_min = 0.0f, range_max = 0.0f;
  std::vector<float> ranges, intensities;  // intensities may be empty
};

// A return on the frame's x-y plane (its height dropped).
struct Point2 {
  double x = 0.0, y = 0.0;
  float intensity = 0.0f;
};

// The scan's returns placed on the frame by the sensor's pose, in beam order.
std::vector<Point2> scan_points(const ScanData & scan, const Pose3 & pose);

// One 360 deg scan seen from the frame's origin: `bins` sectors from -pi, 2 pi / bins wide, each
// keeping its nearest point within [range_min, range_max]; +inf where none.
void merge_points(const std::vector<Point2> & points, size_t bins, double range_min, double range_max,
                  std::vector<float> & ranges, std::vector<float> & intensities);

// A move on the plane: p -> R(yaw) p + (x, y).
struct Pose2 {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

// How `moving` lies against `reference` (point-to-line ICP on the plane): `correction` takes
// `moving` onto it. `degenerate` when the surroundings do not pin all of x, y and yaw (a corridor,
// one wall), then `correction` means nothing; `weakest` is that least-pinned direction's share
// (the Hessian's smallest eigenvalue per match).
struct Alignment {
  bool converged = false, degenerate = true;
  Pose2 correction;
  size_t matches = 0;     // point pairs within 5 cm at the end
  double rms = 0.0;       // m, of those pairs' point-to-line distances
  double weakest = 0.0;
};
inline constexpr size_t kMinMatches = 100;
inline constexpr double kMinWeakest = 0.05;
Alignment align_points(const std::vector<Point2> & reference, const std::vector<Point2> & moving);

}  // namespace rby1_tools
