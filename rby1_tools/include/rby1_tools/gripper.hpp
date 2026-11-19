// What gripper_driver computes, apart from ROS: where a finger is after one step toward
// its commanded opening (virtual), whether a command can be taken, the opening a 0..1000
// position stands for, and whether homing found the whole travel.
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace rby1_tools {

// The opening after `dt` s moving from `now` toward `goal` at `speed` (m/s), without
// passing it.
inline double step_toward(double now, double goal, double speed, double dt) {
  const double step = speed * dt;
  return goal > now ? std::min(goal, now + step) : std::max(goal, now - step);
}

// Why `opening` (m, each finger) cannot be a command for a gripper opening up to
// `max_opening`, or nothing when it can.
inline std::optional<std::string> refused(double opening, double max_opening) {
  if (!std::isfinite(opening)) return "not a number";
  if (opening < 0.0 || opening > max_opening) {
    return "outside 0 (closed) .. " + std::to_string(max_opening) + " m (open)";
  }
  return std::nullopt;
}

// The least travel (rad) a homed motor must span between its ends: SDK example 35's check
// (v0.10.1), 90 % of the high-speed gripper's 230° (the older gripper's 540° passes too).
inline constexpr double kMinTravel = 230.0 * 0.9 * 3.14159265358979323846 / 180.0;

inline bool spans_travel(double min, double max) { return max - min >= kMinTravel; }

// Each finger's opening (m) at `position`: 0 closed .. 1000 open to `max_opening`.
// `what` names the setting in the error.
inline double opening_at(long position, double max_opening, const std::string & what) {
  if (position < 0 || position > 1000) {
    throw std::invalid_argument(what + " is 0 (closed) .. 1000 (open), got " + std::to_string(position));
  }
  return max_opening * static_cast<double>(position) / 1000.0;
}

}  // namespace rby1_tools
