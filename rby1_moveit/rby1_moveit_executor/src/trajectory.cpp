#include "rby1_moveit_executor/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace rby1_moveit_executor {

double seconds(const builtin_interfaces::msg::Duration & stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}

builtin_interfaces::msg::Duration stamp(double t) {
  const auto total = static_cast<int64_t>(std::llround(t * 1e9));
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<int32_t>(total / 1000000000);
  result.nanosec = static_cast<uint32_t>(total % 1000000000);
  return result;
}

std::vector<double> times_of(const JointTrajectory & trajectory) {
  std::vector<double> times;
  for (const auto & point : trajectory.points) times.push_back(seconds(point.time_from_start));
  return times;
}

Timing move_duration(double natural, double duration, double minimum_time) {
  if (duration > 0) return {duration, "duration"};
  if (natural < minimum_time) return {minimum_time, "minimum_time"};
  return {natural, "moveit"};
}

namespace {

double sign(double v) { return (v > 0) - (v < 0); }

// scipy's _edge_case: a one-sided three-point estimate, kept shape-preserving.
double edge(double h0, double h1, double m0, double m1) {
  double d = ((2 * h0 + h1) * m0 - h0 * m1) / (h0 + h1);
  if (sign(d) != sign(m0)) {
    d = 0.0;
  } else if (sign(m0) != sign(m1) && std::abs(d) > std::abs(3 * m0)) {
    d = 3 * m0;
  }
  return d;
}

}  // namespace

void pchip(const std::vector<double> & x, const std::vector<double> & y, const std::vector<double> & at,
           std::vector<double> & value, std::vector<double> & slope) {
  const size_t n = x.size();
  std::vector<double> h(n - 1), m(n - 1), d(n);
  for (size_t k = 0; k + 1 < n; ++k) {
    h[k] = x[k + 1] - x[k];
    m[k] = (y[k + 1] - y[k]) / h[k];
  }
  if (n == 2) {
    d[0] = d[1] = m[0];
  } else {
    for (size_t k = 1; k + 1 < n; ++k) {
      if (m[k - 1] * m[k] <= 0) {
        d[k] = 0.0;
      } else {
        const double w1 = 2 * h[k] + h[k - 1], w2 = h[k] + 2 * h[k - 1];
        d[k] = (w1 + w2) / (w1 / m[k - 1] + w2 / m[k]);
      }
    }
    d[0] = edge(h[0], h[1], m[0], m[1]);
    d[n - 1] = edge(h[n - 2], h[n - 3], m[n - 2], m[n - 3]);
  }
  value.resize(at.size());
  slope.resize(at.size());
  for (size_t i = 0; i < at.size(); ++i) {
    // The segment [x[k], x[k+1]] holding at[i]; the end segments extend past the ends.
    const auto after = std::upper_bound(x.begin(), x.end(), at[i]) - x.begin();
    const size_t k = static_cast<size_t>(std::clamp<long>(after - 1, 0, static_cast<long>(n) - 2));
    const double t = (at[i] - x[k]) / h[k];
    const double t2 = t * t, t3 = t2 * t;
    value[i] = (2 * t3 - 3 * t2 + 1) * y[k] + (t3 - 2 * t2 + t) * h[k] * d[k] +
               (-2 * t3 + 3 * t2) * y[k + 1] + (t3 - t2) * h[k] * d[k + 1];
    slope[i] = ((6 * t2 - 6 * t) * y[k] + (3 * t2 - 4 * t + 1) * h[k] * d[k] +
                (-6 * t2 + 6 * t) * y[k + 1] + (3 * t2 - 2 * t) * h[k] * d[k + 1]) / h[k];
  }
}

JointTrajectory retime(const JointTrajectory & trajectory, double duration, double step, const Bounds & bounds) {
  if (!(step >= 0.001 && step <= kMaxStep)) {
    std::ostringstream text;
    text << "step must be in [0.001, " << kMaxStep << "] s";
    throw std::invalid_argument(text.str());
  }
  if (!(std::isfinite(duration) && duration > 0)) {
    throw std::invalid_argument("duration must be a positive number of seconds");
  }
  const auto times = times_of(trajectory);
  if (times.size() < 2 || std::adjacent_find(times.begin(), times.end(),
                                             [](double a, double b) { return b <= a; }) != times.end()) {
    throw std::invalid_argument("planned trajectory needs at least two strictly increasing timestamps");
  }
  std::vector<double> scaled(times.size());
  for (size_t k = 0; k < times.size(); ++k) {
    scaled[k] = (times[k] - times[0]) * (duration / (times.back() - times[0]));
  }
  const size_t count = std::max<size_t>(2, static_cast<size_t>(std::ceil(duration / step)) + 1);
  std::vector<double> samples(count);
  for (size_t i = 0; i < count; ++i) samples[i] = duration * static_cast<double>(i) / (count - 1);

  const size_t joints = trajectory.joint_names.size();
  JointTrajectory result;
  result.joint_names = trajectory.joint_names;
  result.points.resize(count);
  for (size_t i = 0; i < count; ++i) {
    result.points[i].positions.resize(joints);
    result.points[i].velocities.resize(joints);
    result.points[i].time_from_start = stamp(samples[i]);
  }
  for (size_t j = 0; j < joints; ++j) {
    std::vector<double> y(times.size()), q, qd;
    for (size_t k = 0; k < times.size(); ++k) y[k] = trajectory.points[k].positions[j];
    pchip(scaled, y, samples, q, qd);
    q.front() = y.front();
    q.back() = y.back();
    qd.front() = qd.back() = 0.0;
    const auto limit = bounds.find(trajectory.joint_names[j]);
    for (size_t i = 0; i < count; ++i) {
      double position = q[i];
      if (limit != bounds.end()) {
        position = std::clamp(position, limit->second.first + kLimitMargin, limit->second.second - kLimitMargin);
      }
      result.points[i].positions[j] = position;
      result.points[i].velocities[j] = qd[i];
    }
  }
  return result;
}

std::pair<double, std::string> peak_speed_ratio(const JointTrajectory & trajectory,
                                                const std::map<std::string, double> & limits) {
  double worst = 0.0;
  std::string name;
  for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
    const auto limit = limits.find(trajectory.joint_names[j]);
    if (limit == limits.end() || limit->second <= 0) continue;
    double peak = 0.0;
    for (const auto & point : trajectory.points) peak = std::max(peak, std::abs(point.velocities[j]));
    if (peak / limit->second > worst) {
      worst = peak / limit->second;
      name = trajectory.joint_names[j];
    }
  }
  return {worst, name};
}

JointTrajectory with_hold(const JointTrajectory & trajectory, double hold, double step) {
  if (hold <= 0) return trajectory;
  JointTrajectory held = trajectory;
  const double end = seconds(trajectory.points.back().time_from_start);
  const auto & final = trajectory.points.back().positions;
  const int count = std::max(1, static_cast<int>(std::ceil(hold / step)));
  for (int i = 1; i <= count; ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = final;
    point.velocities.assign(final.size(), 0.0);
    point.time_from_start = stamp(end + hold * i / count);
    held.points.push_back(point);
  }
  return held;
}

Eigen::Matrix4d check_transform(const std::vector<double> & values) {
  if (values.size() != 16) {
    throw std::invalid_argument("expected 16 values (row-major 4x4), got " + std::to_string(values.size()));
  }
  Eigen::Matrix4d matrix;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) matrix(r, c) = values[r * 4 + c];
  }
  if (!matrix.allFinite()) throw std::invalid_argument("non-finite values");
  if ((matrix.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff() > 1e-6) {
    throw std::invalid_argument("bottom row must be [0 0 0 1] -- is it transposed?");
  }
  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  if ((rotation * rotation.transpose() - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1e-3 ||
      std::abs(rotation.determinant() - 1) >= 1e-3) {
    throw std::invalid_argument("the rotation part is not a proper rotation");
  }
  return matrix;
}

}  // namespace rby1_moveit_executor
