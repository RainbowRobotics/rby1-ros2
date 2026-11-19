#include "rby1_tools/lidar_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include <Eigen/Dense>

namespace rby1_tools {

namespace {

constexpr size_t kBlocks = 12, kReturnsPerBlock = 16, kBlockSize = 4 + kReturnsPerBlock * 6;
constexpr uint16_t kDataFlag = 0xEEFF;  // the bytes FF EE, read little endian
constexpr int kFullTurn = 36000;        // 0.01 deg

uint16_t word(const uint8_t * p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }

}  // namespace

std::vector<LidarReturn> parse_lakibeam(const uint8_t * data, size_t size, uint16_t & resolution) {
  std::vector<LidarReturn> returns;
  if (size != kLakibeamPacketSize) return returns;
  const int step = (word(data + kBlockSize + 2) - word(data + 2) + kFullTurn) % kFullTurn;  // across 0 too
  if (word(data) == kDataFlag && word(data + kBlockSize) == kDataFlag && step > 0) {
    resolution = static_cast<uint16_t>(step / kReturnsPerBlock);
  }
  if (resolution == 0) return returns;
  returns.reserve(kBlocks * kReturnsPerBlock);
  for (size_t b = 0; b < kBlocks; ++b) {
    const uint8_t * block = data + b * kBlockSize;
    if (word(block) != kDataFlag) continue;
    const uint16_t azimuth = word(block + 2);
    for (size_t i = 0; i < kReturnsPerBlock; ++i) {
      const uint8_t * result = block + 4 + i * 6;
      LidarReturn r;
      r.azimuth = static_cast<uint16_t>((azimuth + resolution * i) % kFullTurn);
      r.distance = word(result);
      r.intensity = result[2];
      returns.push_back(r);
    }
  }
  return returns;
}

void ScanAssembler::reset(uint16_t resolution) {
  resolution_ = resolution;
  const size_t bins = static_cast<size_t>(kFullTurn / resolution);
  building_ranges_.assign(bins, std::numeric_limits<float>::infinity());
  building_intensities_.assign(bins, 0.0f);
}

bool ScanAssembler::add(const std::vector<LidarReturn> & returns, uint16_t resolution) {
  if (resolution == 0) return false;
  if (resolution != resolution_) {  // a new rate: start over
    reset(resolution);
    started_ = false;
    last_azimuth_ = -1;
  }
  bool finished = false;
  for (const auto & r : returns) {
    if (last_azimuth_ >= 0 && r.azimuth < last_azimuth_) {  // back past azimuth 0: a turn ended
      if (started_) {
        ranges_.swap(building_ranges_);
        intensities_.swap(building_intensities_);
        finished = true;
      }
      started_ = true;
      reset(resolution);
    }
    last_azimuth_ = r.azimuth;
    const size_t bin = r.azimuth / resolution;
    if (started_ && bin < building_ranges_.size() && r.distance > 0) {
      building_ranges_[bin] = static_cast<float>(r.distance) / 1000.0f;
      building_intensities_[bin] = r.intensity;
    }
  }
  return finished;
}

Pose3 pose_from_xyz_rpy(const std::array<double, 3> & xyz, const std::array<double, 3> & rpy) {
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]), cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  Pose3 pose;  // Rz(yaw) * Ry(pitch) * Rx(roll), as tf2's setRPY
  pose.rotation = {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                   sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                   -sp,     cp * sr,                cp * cr};
  pose.translation = xyz;
  return pose;
}

std::array<double, 3> rpy_of(const Pose3 & pose) {
  const auto & r = pose.rotation;
  return {std::atan2(r[7], r[8]), std::asin(std::clamp(-r[6], -1.0, 1.0)), std::atan2(r[3], r[0])};
}

std::vector<Point2> scan_points(const ScanData & scan, const Pose3 & pose) {
  std::vector<Point2> points;
  points.reserve(scan.ranges.size());
  const auto & r = pose.rotation;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range <= 0.0f || range < scan.range_min || range > scan.range_max) continue;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double sx = range * std::cos(angle), sy = range * std::sin(angle);
    Point2 p;
    p.x = r[0] * sx + r[1] * sy + pose.translation[0];
    p.y = r[3] * sx + r[4] * sy + pose.translation[1];
    p.intensity = i < scan.intensities.size() ? scan.intensities[i] : 0.0f;
    points.push_back(p);
  }
  return points;
}

void merge_points(const std::vector<Point2> & points, size_t bins, double range_min, double range_max,
                  std::vector<float> & ranges, std::vector<float> & intensities) {
  ranges.assign(bins, std::numeric_limits<float>::infinity());
  intensities.assign(bins, 0.0f);
  if (bins == 0) return;
  for (const auto & p : points) {
    const double distance = std::hypot(p.x, p.y);
    if (distance < range_min || distance > range_max) continue;
    auto bin = static_cast<size_t>((std::atan2(p.y, p.x) + M_PI) / (2.0 * M_PI) * static_cast<double>(bins));
    if (bin >= bins) bin = 0;  // atan2 gives +pi too, the same direction as -pi
    if (distance < ranges[bin]) {
      ranges[bin] = static_cast<float>(distance);
      intensities[bin] = p.intensity;
    }
  }
}

namespace {

// The line through a point and its neighbours in beam order (within 0.25 m), by its normal; none
// where they do not lie on a line (a corner, an edge, a lone point).
struct Normal {
  double x = 0.0, y = 0.0;
  bool ok = false;
};

std::vector<Normal> line_normals(const std::vector<Point2> & points) {
  constexpr int kSide = 5;
  constexpr double kReach = 0.25, kMaxFlatness = 0.3;
  std::vector<Normal> normals(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    double mx = 0.0, my = 0.0;
    std::vector<size_t> near;
    for (int k = -kSide; k <= kSide; ++k) {
      const long j = static_cast<long>(i) + k;
      if (j < 0 || j >= static_cast<long>(points.size())) continue;
      if (std::hypot(points[j].x - points[i].x, points[j].y - points[i].y) > kReach) continue;
      near.push_back(static_cast<size_t>(j));
      mx += points[j].x;
      my += points[j].y;
    }
    if (near.size() < 4) continue;
    mx /= near.size();
    my /= near.size();
    Eigen::Matrix2d c = Eigen::Matrix2d::Zero();
    for (const size_t j : near) {
      const Eigen::Vector2d d(points[j].x - mx, points[j].y - my);
      c += d * d.transpose();
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(c);
    if (eig.eigenvalues()(1) <= 0.0 || eig.eigenvalues()(0) / eig.eigenvalues()(1) > kMaxFlatness) continue;
    normals[i] = {eig.eigenvectors()(0, 0), eig.eigenvectors()(1, 0), true};
  }
  return normals;
}

// The reference points with a line, in square cells as wide as the largest pair distance.
class Grid {
public:
  Grid(const std::vector<Point2> & points, const std::vector<Normal> & normals, double cell)
      : points_(points), cell_(cell) {
    for (size_t i = 0; i < points.size(); ++i) {
      if (normals[i].ok) cells_[key(cell_of(points[i].x), cell_of(points[i].y))].push_back(i);
    }
  }

  // The nearest within `reach` (<= the cell width) of (x, y), or -1.
  long nearest(double x, double y, double reach) const {
    long best = -1;
    double best_d2 = reach * reach;
    const long cx = cell_of(x), cy = cell_of(y);
    for (long dx = -1; dx <= 1; ++dx) {
      for (long dy = -1; dy <= 1; ++dy) {
        const auto found = cells_.find(key(cx + dx, cy + dy));
        if (found == cells_.end()) continue;
        for (const size_t i : found->second) {
          const double d2 = (points_[i].x - x) * (points_[i].x - x) + (points_[i].y - y) * (points_[i].y - y);
          if (d2 < best_d2) {
            best_d2 = d2;
            best = static_cast<long>(i);
          }
        }
      }
    }
    return best;
  }

private:
  long cell_of(double v) const { return static_cast<long>(std::floor(v / cell_)); }
  static int64_t key(long x, long y) { return (static_cast<int64_t>(x) << 32) ^ (static_cast<int64_t>(y) & 0xffffffff); }

  const std::vector<Point2> & points_;
  double cell_;
  std::unordered_map<int64_t, std::vector<size_t>> cells_;
};

}  // namespace

Alignment align_points(const std::vector<Point2> & reference, const std::vector<Point2> & moving) {
  // Coarse to fine: pairs up to 50 cm apart first, so that a few cm and degrees off still match.
  constexpr std::array<double, 4> kReach{0.5, 0.25, 0.1, 0.05};
  constexpr int kMaxSteps = 30;
  const auto normals = line_normals(reference);
  Alignment result;
  Pose2 & t = result.correction;
  Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
  for (const double reach : kReach) {
    const Grid grid(reference, normals, reach);
    result.converged = false;
    for (int step = 0; step < kMaxSteps; ++step) {
      h.setZero();
      Eigen::Vector3d g = Eigen::Vector3d::Zero();
      double squares = 0.0;
      size_t matches = 0;
      const double c = std::cos(t.yaw), s = std::sin(t.yaw);
      for (const auto & p : moving) {
        const double x = c * p.x - s * p.y + t.x, y = s * p.x + c * p.y + t.y;
        const long i = grid.nearest(x, y, reach);
        if (i < 0) continue;
        const Normal & n = normals[static_cast<size_t>(i)];
        const double r = n.x * (x - reference[i].x) + n.y * (y - reference[i].y);
        const Eigen::Vector3d j(n.x, n.y, n.x * -y + n.y * x);  // d r / d (x, y, yaw) of a small move after t
        h += j * j.transpose();
        g += j * r;
        squares += r * r;
        ++matches;
      }
      result.matches = matches;
      result.rms = matches > 0 ? std::sqrt(squares / matches) : 0.0;
      if (matches < 3) return result;  // nothing to match: not converged
      const Eigen::Vector3d d = h.ldlt().solve(-g);
      if (!d.allFinite()) return result;
      const double dc = std::cos(d(2)), ds = std::sin(d(2));
      t = {dc * t.x - ds * t.y + d(0), ds * t.x + dc * t.y + d(1), t.yaw + d(2)};
      if (std::hypot(d(0), d(1)) < 1e-6 && std::abs(d(2)) < 1e-7) {
        result.converged = true;
        break;
      }
    }
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(h);
  result.weakest = result.matches > 0 ? eig.eigenvalues()(0) / static_cast<double>(result.matches) : 0.0;
  result.degenerate = result.matches < kMinMatches || result.weakest < kMinWeakest;
  return result;
}

}  // namespace rby1_tools
