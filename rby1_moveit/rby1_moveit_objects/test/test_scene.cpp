// Obstacle specs, files, slides and the command line; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "rby1_moveit_objects/command.hpp"
#include "rby1_moveit_objects/object_spec.hpp"

using namespace rby1_moveit_objects;
using shape_msgs::msg::SolidPrimitive;

namespace {

ObjectSpec spec(const std::string & name, const std::string & type) {
  ObjectSpec s;
  s.name = name;
  s.type = type;
  s.xyz = {0.0, 0.0, 1.0};
  return s;
}

std::vector<double> dimensions(const moveit_msgs::msg::CollisionObject & object) {
  const auto & d = object.primitives[0].dimensions;
  return std::vector<double>(d.begin(), d.end());
}

std::string failure(const ObjectSpec & s) {
  try {
    collision_object(s);
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(ObjectSpec, BoxSphereAndCylinderBecomeMoveItPrimitives) {
  auto box = spec("b", "box");
  box.xyz = {1, 2, 3};
  box.size = {0.1, 0.2, 0.3};
  auto sphere = spec("s", "sphere");
  sphere.radius = 0.05;
  auto cylinder = spec("c", "cylinder");
  cylinder.radius = 0.04;
  cylinder.height = 0.6;
  const auto b = collision_object(box), s = collision_object(sphere), c = collision_object(cylinder);
  EXPECT_EQ(b.primitives[0].type, SolidPrimitive::BOX);
  EXPECT_EQ(dimensions(b), (std::vector<double>{0.1, 0.2, 0.3}));
  EXPECT_EQ(s.primitives[0].type, SolidPrimitive::SPHERE);
  // MoveIt's cylinder dimensions are [height, radius], in that order.
  EXPECT_EQ(dimensions(c), (std::vector<double>{0.6, 0.04}));
  EXPECT_EQ(b.header.frame_id, "base");
  EXPECT_DOUBLE_EQ(b.primitive_poses[0].position.x, 1.0);
  EXPECT_DOUBLE_EQ(b.primitive_poses[0].position.z, 3.0);
}

TEST(ObjectSpec, FrameIsHonoured) {
  auto box = spec("b", "box");
  box.size = {1, 1, 1};
  box.frame = "link_torso_5";
  EXPECT_EQ(collision_object(box).header.frame_id, "link_torso_5");
}

TEST(ObjectSpec, MalformedObjectsAreRejectedWithAReason) {
  auto cone = spec("x", "cone");
  auto unnamed = spec("", "box");
  unnamed.size = {1, 1, 1};
  auto no_size = spec("x", "box");
  auto no_height = spec("x", "cylinder");
  no_height.radius = 0.1;
  auto no_xyz = spec("x", "sphere");
  no_xyz.radius = 0.1;
  no_xyz.xyz.clear();
  auto short_size = spec("x", "box");
  short_size.size = {1, 1};
  auto negative = spec("x", "sphere");
  negative.radius = -1;
  auto nan = spec("x", "sphere");
  nan.radius = 1;
  nan.xyz = {0, 0, NAN};
  const std::vector<std::pair<ObjectSpec, std::string>> cases = {
    {cone, "type must be"}, {unnamed, "name"}, {no_size, "needs size"}, {no_height, "needs height"},
    {no_xyz, "needs xyz"}, {short_size, "3 values"}, {negative, "positive"}, {nan, "finite"}};
  for (const auto & [bad, message] : cases) {
    EXPECT_NE(failure(bad).find(message), std::string::npos) << "expected '" << message << "', got '"
                                                             << failure(bad) << "'";
  }
}

TEST(ObjectSpec, RpyBecomesAUnitQuaternionMatchingItsRotation) {
  const std::vector<std::array<double, 3>> cases = {{0, 0, 0}, {0.3, -0.2, 1.1}, {M_PI, 0, 0}};
  for (const auto & rpy : cases) {
    const auto [x, y, z, w] = quaternion_from_rpy(rpy[0], rpy[1], rpy[2]);
    EXPECT_NEAR(x * x + y * y + z * z + w * w, 1.0, 1e-12);
    // Rz(yaw) Ry(pitch) Rx(roll), column by column, against the quaternion's matrix.
    const double r = rpy[0], p = rpy[1], yw = rpy[2];
    const double expected[3][3] = {
      {std::cos(yw) * std::cos(p), std::cos(yw) * std::sin(p) * std::sin(r) - std::sin(yw) * std::cos(r),
       std::cos(yw) * std::sin(p) * std::cos(r) + std::sin(yw) * std::sin(r)},
      {std::sin(yw) * std::cos(p), std::sin(yw) * std::sin(p) * std::sin(r) + std::cos(yw) * std::cos(r),
       std::sin(yw) * std::sin(p) * std::cos(r) - std::cos(yw) * std::sin(r)},
      {-std::sin(p), std::cos(p) * std::sin(r), std::cos(p) * std::cos(r)}};
    const double got[3][3] = {
      {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
      {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
      {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) EXPECT_NEAR(got[i][j], expected[i][j], 1e-12);
    }
  }
}

TEST(ObjectFile, TheShippedExampleFileLoads) {
  const auto specs = specs_from_file(EXAMPLE_SCENE);
  ASSERT_EQ(specs.size(), 3u);
  EXPECT_EQ(specs[0].name, "table");
  EXPECT_EQ(specs[1].name, "post");
  EXPECT_EQ(specs[2].name, "ball");
}

TEST(ObjectFile, AnEmptyFileIsRefused) {
  const std::string path = testing::TempDir() + "empty_scene.yaml";
  std::ofstream(path) << "objects: []\n";
  try {
    specs_from_file(path);
    FAIL() << "an empty file was accepted";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("non-empty"), std::string::npos);
  }
  std::remove(path.c_str());
}

TEST(ObjectFile, OneBadEntryRefusesTheFile) {
  const std::string path = testing::TempDir() + "bad_scene.yaml";
  std::ofstream(path) << "objects:\n  - {name: a, type: box, xyz: [0, 0, 1], size: [1, 1, 1]}\n"
                         "  - {name: b, type: sphere, xyz: [0, 0, 1]}\n";
  EXPECT_THROW(specs_from_file(path), std::invalid_argument);
  std::remove(path.c_str());
}

TEST(Move, AMovedObjectSlidesThenStaysWhereItEnds) {
  const std::array<double, 3> start{0.3, -0.6, 1.2}, velocity{0.0, 0.1, 0.0};
  EXPECT_NEAR(moved(start, velocity, 0.0, 3.0)[1], -0.6, 1e-12);
  EXPECT_NEAR(moved(start, velocity, 1.5, 3.0)[1], -0.45, 1e-12);
  EXPECT_NEAR(moved(start, velocity, 10.0, 3.0)[1], -0.3, 1e-12);
  EXPECT_DOUBLE_EQ(moved(start, velocity, 10.0, 3.0)[2], 1.2);
}

TEST(Move, BadMovesAreRefused) {
  EXPECT_THROW(check_move({0.0, 0.1}, 3.0, 10.0), std::invalid_argument);
  EXPECT_THROW(check_move({0.0, NAN, 0.0}, 3.0, 10.0), std::invalid_argument);
  EXPECT_THROW(check_move({0.0, 0.1, 0.0}, 0.0, 10.0), std::invalid_argument);
  EXPECT_THROW(check_move({0.0, 0.1, 0.0}, 3.0, 100.0), std::invalid_argument);
  EXPECT_NO_THROW(check_move({0.0, 0.1, 0.0}, 3.0, 10.0));
}

TEST(CommandLine, Move) {
  const auto command = parse_command({"move", "probe", "--velocity", "0", "0.1", "0", "--time", "6"});
  EXPECT_EQ(command.name, "move");
  EXPECT_EQ(command.names, std::vector<std::string>{"probe"});
  EXPECT_EQ(command.velocity, (std::vector<double>{0.0, 0.1, 0.0}));
  EXPECT_DOUBLE_EQ(command.time, 6.0);
  EXPECT_DOUBLE_EQ(command.rate, 10.0);
}

TEST(CommandLine, AddCylinderInAFrame) {
  const auto command = parse_command({"add", "cylinder", "post", "--xyz", "0.5", "-0.5", "0.9", "--radius",
                                      "0.04", "--height", "0.6", "--frame", "link_torso_5"});
  EXPECT_EQ(command.spec.type, "cylinder");
  EXPECT_EQ(command.spec.name, "post");
  EXPECT_EQ(command.spec.xyz, (std::vector<double>{0.5, -0.5, 0.9}));
  EXPECT_DOUBLE_EQ(*command.spec.height, 0.6);
  EXPECT_EQ(command.spec.frame, "link_torso_5");
  EXPECT_NO_THROW(collision_object(command.spec));
}

TEST(CommandLine, MistakesAreRefusedBeforeTouchingROS) {
  EXPECT_THROW(parse_command({}), std::invalid_argument);
  EXPECT_THROW(parse_command({"fly"}), std::invalid_argument);
  EXPECT_THROW(parse_command({"add", "box", "b", "--size", "1", "1", "1"}), std::invalid_argument);  // no xyz
  EXPECT_THROW(parse_command({"add", "box", "b", "--xyz", "1", "x", "1"}), std::invalid_argument);
  EXPECT_THROW(parse_command({"move", "probe", "--time", "3"}), std::invalid_argument);
  EXPECT_THROW(parse_command({"remove"}), std::invalid_argument);
  EXPECT_EQ(parse_command({"--help"}).name, "help");
  EXPECT_EQ(parse_command({"remove", "a", "b"}).names, (std::vector<std::string>{"a", "b"}));
}
