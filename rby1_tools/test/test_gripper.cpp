// The gripper's virtual step, command check and 0..1000 positions; no ROS graph.
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "rby1_tools/gripper.hpp"

using namespace rby1_tools;

TEST(Step, MovesAtSpeedAndStopsAtTheGoal) {
  EXPECT_NEAR(step_toward(0.05, 0.0, 0.05, 0.02), 0.049, 1e-12);  // closing, 1 mm per 20 ms
  EXPECT_NEAR(step_toward(0.0, 0.05, 0.05, 0.02), 0.001, 1e-12);  // opening
  EXPECT_EQ(step_toward(0.0005, 0.0, 0.05, 0.02), 0.0);           // does not pass the goal
  EXPECT_EQ(step_toward(0.03, 0.03, 0.05, 0.02), 0.03);
  double opening = 0.05;
  int steps = 0;
  while (opening > 0.0 && steps < 1000) {
    opening = step_toward(opening, 0.0, 0.05, 0.02);
    ++steps;
  }
  EXPECT_NEAR(steps, 50, 1);  // fully open to closed in 1 s (one more for rounding)
}

TEST(Command, OnlyOpeningsTheGripperHas) {
  EXPECT_FALSE(refused(0.0, 0.05));
  EXPECT_FALSE(refused(0.05, 0.05));
  EXPECT_TRUE(refused(-0.001, 0.05));
  EXPECT_TRUE(refused(0.051, 0.05));
  EXPECT_TRUE(refused(std::numeric_limits<double>::quiet_NaN(), 0.05));
}

TEST(Position, ZeroIsClosedAndAThousandIsOpen) {
  EXPECT_EQ(opening_at(0, 0.05, "initial_position"), 0.0);
  EXPECT_NEAR(opening_at(500, 0.05, "initial_position"), 0.025, 1e-12);
  EXPECT_NEAR(opening_at(1000, 0.05, "initial_position"), 0.05, 1e-12);
  EXPECT_THROW(opening_at(-1, 0.05, "initial_position"), std::invalid_argument);
  EXPECT_THROW(opening_at(1001, 0.05, "open_position"), std::invalid_argument);
}

TEST(Homing, MustSpanTheTravel) {
  const double deg = 3.14159265358979323846 / 180.0;
  EXPECT_TRUE(spans_travel(0.0, 230.0 * deg));   // the high-speed gripper
  EXPECT_TRUE(spans_travel(0.0, 540.0 * deg));   // the older one
  EXPECT_FALSE(spans_travel(0.0, 200.0 * deg));  // one end only
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(spans_travel(inf, -inf));         // never read
}
