// Values the RB-Y1 driver fixes in code (2026-10-30). None of them is a key of
// driver_parameters.yaml: what a user may set is DriverParameters in type.hpp. They are
// gathered here so that changing one is one line, with the reason each has its value.
#pragma once

#include <chrono>

namespace rby1_ros2 {

// ---- the state loop ------------------------------------------------------------------
constexpr double kStatePeriod = 0.01;   // s; the robot gives its state at 100 Hz
// Seconds of consecutive failed state reads before the robot counts as lost. A network
// that drops out this long is a fault to fix, not a setting to raise.
constexpr double kStateLossTimeout = 1.0;
constexpr auto kStateRetryDelay = std::chrono::milliseconds(5);   // between two tries of one read
constexpr int kNearestLinkPairs = 20;   // link pairs the collision check reports (the nearest)

// ---- power, servo, control manager ---------------------------------------------------
constexpr auto kServoSettleTime = std::chrono::milliseconds(500);  // after servo on, before the info is read
constexpr int kControlReadyTimeoutMs = 2000;   // after enabling, waited for the robot to take commands
// After cancelling, the robot ends the control asynchronously; the control manager refuses to
// disable until it is idle: polled this many times, this far apart (2 s in all).
constexpr int kControlIdleTries = 40;
constexpr auto kControlIdleStep = std::chrono::milliseconds(50);

// ---- single commands and their defaults ----------------------------------------------
constexpr double kDefaultMinimumTime = 2.0;         // s, a command that leaves minimum_time at 0
constexpr double kLinearVelocityLimit = 1.5;        // m/s, a Cartesian command that gives none
constexpr double kAngularVelocityLimit = 4.712388;  // rad/s, likewise
constexpr double kAccelerationLimit = 1.0;          // scaling 0 to 1, likewise
constexpr double kTranslationWeight = 1000.0;  // Cartesian commands and the local IK: how much the
constexpr double kRotationWeight = 100.0;      // position and the orientation of the hand count
constexpr double kPostureWeight = 1.0;         // ... against staying near the current posture (IK)
constexpr double kDefaultJointStiffness = 100.0;  // N·m/rad, joint impedance when the request gives none
constexpr double kDefaultDampingRatio = 1.0;      // likewise
constexpr double kDefaultTorqueLimit = 10.0;      // N·m, likewise
constexpr double kMobilityHoldTime = 1.0;      // s, a velocity command outlives its sender this long

// ---- the command streams -------------------------------------------------------------
// A streamed command holds its part until the channel is closed: the SDK's control hold
// time is set to this (seconds), far beyond any session.
constexpr double kControlHoldTime = 1e6;
constexpr double kCommandTimeFloor = 0.01;   // s, a trajectory segment or first point given 0 time
constexpr double kStreamCartesianMinimumTime = 0.1;   // s, a stream_cartesian target given none
// A trajectory's first waypoint may need at most this share of a joint's velocity limit
// to be reached from the measured posture in its time.
constexpr double kFjtStartVelocityScale = 1.0;
constexpr double kStreamSendWait = 0.5;   // s, at most, for a channel being reopened before a send
constexpr int kFjtMaxFailedSends = 3;     // waypoints the robot did not get, in a row, end the trajectory
// A stream channel opened but never commanded is closed after this long. Once commanded
// it stays open: its last command holds its part (kControlHoldTime) until it is closed.
constexpr double kStreamIdleTimeout = 60.0;   // s
// A stream opened within this of closing one takes commands the robot ignores.
constexpr auto kStreamReopenGap = std::chrono::milliseconds(300);
constexpr auto kStreamWaitStep = std::chrono::milliseconds(20);   // polling a busy channel
constexpr double kCancelHoldTime = 0.2;   // s, the move that holds a cancelled part where it is
constexpr int kStreamResultTimeoutS = 2;  // s, streamed action results are kept (thousands otherwise)

// ---- the local IK (robot_cartesian with a planner-free target) -----------------------
constexpr double kIkStep = 0.1;               // s, one Newton step of the pose
constexpr int kIkMaxIterations = 50;
constexpr double kIkTolerance = 1e-3;         // the solver's own convergence test
constexpr double kIkPositionTolerance = 5e-4;     // m, the pose counts as reached within these
constexpr double kIkOrientationTolerance = 2e-3;  // rad
constexpr double kIkJointLimitMargin = 0.5;   // rad, the solver may pass a joint limit by this
constexpr double kIkVelocityBound = 100.0;    // rad/s, the solver's bounds (not the robot's)
constexpr double kIkAccelerationBound = 1000.0;   // rad/s^2
constexpr double kIkStallStep = 1e-7;         // rad, a step smaller than this ends the search
// Joints whose URDF limit is beyond this (wheels: DBL_MAX) are left out of the solver;
// including them makes the problem ill-conditioned.
constexpr double kFiniteLimitThreshold = 1e6;

}  // namespace rby1_ros2
