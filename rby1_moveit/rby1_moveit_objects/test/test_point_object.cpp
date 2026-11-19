// The object of object_at_point: its parameters checked, and the spec it becomes at a
// point; no ROS graph.
#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

#include "rby1_moveit_objects/point_object.hpp"

using namespace rby1_moveit_objects;
using shape_msgs::msg::SolidPrimitive;

namespace {

ObjectSpec box() {
  ObjectSpec spec;
  spec.name = "marker_box";
  spec.type = "box";
  spec.size = {0.06, 0.08, 0.10};
  return spec;
}

std::string failure(const ObjectSpec & spec, const std::vector<double> & offset = {0.0, 0.0, 0.0}) {
  try {
    point_object(spec, offset);
  } catch (const std::invalid_argument & error) {
    return error.what();
  }
  return "";
}

}  // namespace

TEST(PointObject, ThePointPlusTheOffsetIsTheCentre) {
  const auto object = point_object(box(), {0.0, 0.01, 0.05});
  const auto spec = at_point(object, "link_torso_5", 0.5, -0.3, 1.0);
  EXPECT_EQ(spec.name, "marker_box");
  EXPECT_EQ(spec.frame, "link_torso_5");
  ASSERT_EQ(spec.xyz.size(), 3u);
  EXPECT_DOUBLE_EQ(spec.xyz[0], 0.5);
  EXPECT_DOUBLE_EQ(spec.xyz[1], -0.29);
  EXPECT_DOUBLE_EQ(spec.xyz[2], 1.05);
  const auto placed = collision_object(spec);
  EXPECT_EQ(placed.id, "marker_box");
  EXPECT_EQ(placed.operation, moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_EQ(placed.header.frame_id, "link_torso_5");
  EXPECT_EQ(placed.primitives[0].type, SolidPrimitive::BOX);
  EXPECT_DOUBLE_EQ(placed.primitive_poses[0].position.z, 1.05);
}

TEST(PointObject, AnEmptyFrameMeansBase) {
  const auto object = point_object(box(), {0.0, 0.0, 0.0});
  EXPECT_EQ(at_point(object, "", 0.1, 0.2, 0.3).frame, "base");
  EXPECT_EQ(collision_object(at_point(object, "", 0.1, 0.2, 0.3)).header.frame_id, "base");
}

TEST(PointObject, EveryPointGivesTheSameNameSoItReplacesTheLast) {
  const auto object = point_object(box(), {0.0, 0.0, 0.0});
  const auto first = collision_object(at_point(object, "base", 0.5, 0.0, 1.0));
  const auto second = collision_object(at_point(object, "base", 0.6, 0.1, 1.1));
  EXPECT_EQ(first.id, second.id);
  EXPECT_DOUBLE_EQ(second.primitive_poses[0].position.x, 0.6);
}

TEST(PointObject, SphereCylinderAndRpyAreTaken) {
  ObjectSpec sphere;
  sphere.name = "ball";
  sphere.type = "sphere";
  sphere.radius = 0.03;
  EXPECT_EQ(collision_object(at_point(point_object(sphere, {0, 0, 0}), "", 0, 0, 1)).primitives[0].type,
            SolidPrimitive::SPHERE);
  ObjectSpec cylinder;
  cylinder.name = "post";
  cylinder.type = "cylinder";
  cylinder.radius = 0.02;
  cylinder.height = 0.3;
  cylinder.rpy = {0.0, 0.0, M_PI};
  const auto post = collision_object(at_point(point_object(cylinder, {0, 0, 0.15}), "", 0, 0, 1));
  EXPECT_EQ(post.primitives[0].type, SolidPrimitive::CYLINDER);
  EXPECT_NEAR(post.primitive_poses[0].orientation.z, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(post.primitive_poses[0].position.z, 1.15);
}

TEST(PointObject, BadParametersSayWhatIsWrong) {
  auto cone = box();
  cone.type = "cone";
  auto unnamed = box();
  unnamed.name.clear();
  auto no_size = box();
  no_size.size.clear();
  auto negative = box();
  negative.size = {0.06, -0.06, 0.06};
  ObjectSpec no_radius;
  no_radius.name = "ball";
  no_radius.type = "sphere";
  ObjectSpec no_height = no_radius;
  no_height.type = "cylinder";
  no_height.radius = 0.02;
  auto short_rpy = box();
  short_rpy.rpy = {0.0, 0.0};
  EXPECT_NE(failure(cone).find("kind must be"), std::string::npos) << failure(cone);
  EXPECT_NE(failure(unnamed).find("name"), std::string::npos) << failure(unnamed);
  EXPECT_NE(failure(no_size).find("needs size"), std::string::npos) << failure(no_size);
  EXPECT_NE(failure(negative).find("positive"), std::string::npos) << failure(negative);
  EXPECT_NE(failure(no_radius).find("needs radius"), std::string::npos) << failure(no_radius);
  EXPECT_NE(failure(no_height).find("needs height"), std::string::npos) << failure(no_height);
  EXPECT_NE(failure(short_rpy).find("3 finite values"), std::string::npos) << failure(short_rpy);
  EXPECT_NE(failure(box(), {0.0, 0.0}).find("offset needs 3"), std::string::npos);
  EXPECT_NE(failure(box(), {0.0, NAN, 0.0}).find("offset needs 3"), std::string::npos);
  EXPECT_EQ(failure(box()), "");
}

TEST(PointObject, APointThatIsNotANumberIsRefusedWhenPlaced) {
  const auto object = point_object(box(), {0.0, 0.0, 0.0});
  EXPECT_THROW(collision_object(at_point(object, "base", 0.5, NAN, 1.0)), std::invalid_argument);
}

TEST(PointObject, APoseMovesTheObjectOnlyWhenItIsFurtherThanMinMove) {
  EXPECT_TRUE(moved(std::nullopt, {0.5, -0.3, 1.0}, 0.01));  // the first pose always places it
  const std::optional<std::array<double, 3>> last{{0.5, -0.3, 1.0}};
  EXPECT_FALSE(moved(last, {0.5, -0.3, 1.0}, 0.01));
  EXPECT_FALSE(moved(last, {0.506, -0.306, 1.005}, 0.01));  // 9.8 mm in all
  EXPECT_TRUE(moved(last, {0.506, -0.306, 1.006}, 0.01));   // 10.4 mm
  EXPECT_TRUE(moved(last, {0.5, -0.3, 1.0001}, 0.0));       // min_move 0: every change
  EXPECT_FALSE(moved(last, {0.5, -0.3, 1.0}, 0.0));
}

TEST(PointObject, TheObjectGoesOnlyWhenRemoveAfterIsSetAndThePoseStayedAwayLonger) {
  EXPECT_FALSE(gone(1000.0, 0.0));  // 0: never
  EXPECT_FALSE(gone(1.9, 2.0));
  EXPECT_FALSE(gone(2.0, 2.0));
  EXPECT_TRUE(gone(2.1, 2.0));
}

// The shipped file read as YAML: the node itself takes it as ROS parameters.
TEST(PointObject, TheShippedParameterFileDescribesAValidObject) {
  const YAML::Node parameters = YAML::LoadFile(POINT_OBJECT)["rby1_object_at_point"]["ros__parameters"];
  ASSERT_TRUE(parameters.IsMap());
  ObjectSpec spec;
  spec.name = parameters["name"].as<std::string>();
  spec.type = parameters["kind"].as<std::string>();
  if (parameters["size"]) spec.size = parameters["size"].as<std::vector<double>>();
  if (parameters["radius"]) spec.radius = parameters["radius"].as<double>();
  if (parameters["height"]) spec.height = parameters["height"].as<double>();
  if (parameters["rpy"]) spec.rpy = parameters["rpy"].as<std::vector<double>>();
  const auto offset = parameters["offset"] ? parameters["offset"].as<std::vector<double>>()
                                           : std::vector<double>{0.0, 0.0, 0.0};
  const auto object = point_object(spec, offset);
  EXPECT_EQ(object.spec.name, "point_object");
  EXPECT_NO_THROW(collision_object(at_point(object, "", 0.5, -0.3, 1.0)));
}
