// Copyright 2026 Rainbow Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The scenarios of the web backend's test_config_backup.py, plus files written by the web module itself
// (fixtures/, see fixtures/README.md).

#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rby1_robot_manager/config_backup.hpp"

namespace cb = rby1_robot_manager::config_backup;
using cb::json;

namespace
{

const char * kUrdf = "<robot name='rby1'/>";

// Mimics the SDK as the web test's FakeRobot: JSON text in/out, set_parameter returns false out of range.
class FakeRobot : public cb::ConfigRobot
{
public:
  FakeRobot()
  {
    params = {
      {"robot_model_name", {2, "rby1a"}},
      {"model_name", {2, ""}},
      {"battery_config", {5, json::array({0, 46, 56, 25558.048, -1530.144, 30.294, -0.198})}},
      {"gravity", {3, json::array({0, 0, -9.8})}},
      {"hotspot.ssid", {2, "RBY1"}},
      {"hotspot.password", {2, "RBY1@@@@"}},
      {"default.velocity_limit_scaling", {1, 1.0}},
      {"real_time_control.max_timeout_count", {0, 10}},
      {"boot.power_on_target", {2, ""}},
    };
  }

  Identity identity() override {return {"A", "v1.2", "0.10.7", "0.10.0"};}

  std::vector<std::pair<std::string, int>> parameter_list() override
  {
    std::vector<std::pair<std::string, int>> out;
    for (const auto & [name, p] : params) {
      out.emplace_back(name, p.first);
    }
    return out;
  }

  std::string get_parameter(const std::string & name) override {return params.at(name).second.dump();}

  bool set_parameter(const std::string & name, const std::string & value, bool) override
  {
    set_calls.push_back(name);
    const auto v = json::parse(value);
    if (fail_on.count(name)) {
      return false;
    }
    if (name == "default.velocity_limit_scaling" && (v.get<double>() < 0.01 || v.get<double>() > 1.5)) {
      return false;
    }
    params[name].second = v;
    return true;
  }

  bool import_robot_model(const std::string & name, const std::string & model) override
  {
    if (model.find("bad") != std::string::npos) {
      return false;
    }
    models[name] = model;
    return true;
  }

  std::map<std::string, std::pair<int, json>> params;
  std::set<std::string> fail_on;
  std::vector<std::string> set_calls;
  std::map<std::string, std::string> models;
};

json roundtrip(const json & file) {return cb::parse_file(file.dump());}

json read_fixture(const std::string & name)
{
  std::ifstream in("fixtures/" + name);
  std::stringstream buffer;
  buffer << in.rdbuf();
  return json::parse(buffer.str());
}

std::map<std::string, json> by_name(const json & preview)
{
  std::map<std::string, json> out;
  for (const auto & p : preview["parameters"]) {
    out[p["name"].get<std::string>()] = p;
  }
  return out;
}

class ConfigBackupTest : public ::testing::Test
{
protected:
  FakeRobot robot;
  json local = cb::local_source(robot, "ros-host");
  json file = roundtrip(cb::build_export(robot, local));
};

}  // namespace

TEST_F(ConfigBackupTest, ExportImportRoundtrip)
{
  EXPECT_EQ(file["format"], "rby1-config");
  EXPECT_EQ(file["format_version"], 1);
  EXPECT_EQ(file["parameters"]["gravity"], json({{"type", 3}, {"value", {0, 0, -9.8}}}));
  EXPECT_TRUE(file["port_forward"].is_null());
  EXPECT_TRUE(file["models"].empty());
  EXPECT_EQ(file["source"]["robot_model_name"], "a");
  EXPECT_TRUE(file["source"]["fingerprint"].is_null());
  EXPECT_EQ(cb::export_filename(local).rfind("rby1-config-a-", 0), 0u);

  const auto preview = cb::build_preview(robot, file, local);
  EXPECT_FALSE(preview["same_robot"].get<bool>());  // no fingerprint over gRPC
  for (const auto & p : preview["parameters"]) {
    EXPECT_FALSE(p["changed"].get<bool>()) << p["name"];
  }
}

TEST_F(ConfigBackupTest, PreviewCategoriesAndDefaults)
{
  file["parameters"]["default.velocity_limit_scaling"]["value"] = 0.5;
  file["parameters"]["hotspot.ssid"]["value"] = "OTHER";
  file["parameters"]["robot_model_name"]["value"] = "rby1x";
  file["parameters"]["removed.param"] = {{"type", 1}, {"value", 1.0}};
  file["parameters"]["real_time_control.max_timeout_count"] = {{"type", 1}, {"value", 3.0}};

  const auto preview = cb::build_preview(robot, file, local);
  auto p = by_name(preview);
  EXPECT_TRUE(p["default.velocity_limit_scaling"]["selected"].get<bool>());
  EXPECT_EQ(p["hotspot.ssid"]["category"], "robot_specific");
  EXPECT_TRUE(p["hotspot.ssid"]["selectable"].get<bool>());
  EXPECT_FALSE(p["hotspot.ssid"]["selected"].get<bool>());  // never the same robot over gRPC
  EXPECT_EQ(p["robot_model_name"]["category"], "identity");
  EXPECT_FALSE(p["robot_model_name"]["selectable"].get<bool>());
  EXPECT_EQ(p["removed.param"]["category"], "unknown");
  EXPECT_EQ(p["real_time_control.max_timeout_count"]["category"], "type_mismatch");
  EXPECT_FALSE(p["gravity"]["selected"].get<bool>());
  EXPECT_FALSE(preview["warnings"].empty());

  EXPECT_EQ(cb::default_parameters(preview, false), std::vector<std::string>{"default.velocity_limit_scaling"});
  EXPECT_EQ(
    cb::default_parameters(preview, true),
    (std::vector<std::string>{"default.velocity_limit_scaling", "hotspot.ssid"}));
}

TEST_F(ConfigBackupTest, OtherModelRejected)
{
  file["source"]["robot_model_name"] = "m";
  EXPECT_THROW(cb::build_preview(robot, file, local), cb::ConfigError);
  EXPECT_THROW(cb::apply(robot, file, local, {{"gravity"}, {}}), cb::ConfigError);
}

TEST_F(ConfigBackupTest, InvalidFiles)
{
  const json base = cb::build_export(robot, local);
  std::vector<std::function<void(json &)>> mutations = {
    [](json & f) {f["format"] = "x";},
    [](json & f) {f["format_version"] = 2;},
    [](json & f) {f["parameters"]["gravity"]["value"] = {0, 0};},
    [](json & f) {f["parameters"]["gravity"]["value"] = {0, 0, "a"};},
    [](json & f) {f["parameters"]["real_time_control.max_timeout_count"]["value"] = true;},
    [](json & f) {f["parameters"]["default.velocity_limit_scaling"]["value"] = "1.0";},
    [](json & f) {f["models"]["../etc/passwd"] = kUrdf;},
    [](json & f) {f["models"]["a/b"] = kUrdf;},
    [](json & f) {f["models"]["empty"] = " ";},
    [](json & f) {f["port_forward"] = {{"forwards", "x"}};},
    [](json & f) {f["source"].erase("robot_model_name");},
  };
  for (size_t i = 0; i < mutations.size(); ++i) {
    json f = base;
    mutations[i](f);
    EXPECT_THROW(roundtrip(f), cb::ConfigError) << "mutation " << i;
  }
}

TEST(ConfigBackupFile, TooLargeAndNotJson)
{
  EXPECT_THROW(cb::parse_file(std::string(cb::kMaxFileBytes + 1, ' ')), cb::ConfigError);
  EXPECT_THROW(cb::parse_file("{not json"), cb::ConfigError);
}

TEST_F(ConfigBackupTest, ApplySetsAndSkips)
{
  file["parameters"]["default.velocity_limit_scaling"]["value"] = 0.5;
  file["parameters"]["boot.power_on_target"]["value"] = "48v";
  file["parameters"]["robot_model_name"]["value"] = "rby1x";
  const auto report = cb::apply(
    robot, file, local, {{"default.velocity_limit_scaling", "boot.power_on_target", "robot_model_name"}, {}});
  EXPECT_EQ(report["applied"], json({"default.velocity_limit_scaling", "boot.power_on_target"}));
  EXPECT_EQ(report["skipped"][0]["name"], "robot_model_name");
  EXPECT_EQ(report["restart_required"], json({"boot.power_on_target"}));
  EXPECT_EQ(robot.params["default.velocity_limit_scaling"].second, 0.5);
  EXPECT_EQ(robot.params["robot_model_name"].second, "rby1a");
}

TEST_F(ConfigBackupTest, ApplyRollsBackOnFailure)
{
  file["parameters"]["gravity"]["value"] = {0, 0, -9.81};
  file["parameters"]["default.velocity_limit_scaling"]["value"] = 5.0;  // out of range
  const auto report = cb::apply(robot, file, local, {{"gravity", "default.velocity_limit_scaling"}, {}});
  EXPECT_TRUE(report["rolled_back"].get<bool>());
  EXPECT_TRUE(report["applied"].empty());
  EXPECT_TRUE(report["rollback_errors"].empty());
  EXPECT_EQ(report["failed"][0]["name"], "default.velocity_limit_scaling");
  EXPECT_EQ(robot.params["gravity"].second, json({0, 0, -9.8}));
}

TEST_F(ConfigBackupTest, ModelFailureAndModelName)
{
  file["models"]["broken"] = "bad urdf";
  auto report = cb::apply(robot, file, local, {{}, {"broken"}});
  EXPECT_EQ(report["failed"][0]["name"], "model:broken");
  EXPECT_TRUE(report["rolled_back"].get<bool>());

  // model_name pointing at a model that is not in the file is not selectable / is skipped
  file = roundtrip(cb::build_export(robot, local));
  file["parameters"]["model_name"]["value"] = "ghost";
  auto p = by_name(cb::build_preview(robot, file, local));
  EXPECT_FALSE(p["model_name"]["selectable"].get<bool>());
  report = cb::apply(robot, file, local, {{"model_name"}, {}});
  EXPECT_EQ(report["skipped"][0]["name"], "model_name");
  EXPECT_EQ(robot.params["model_name"].second, "");

  // ...but is applied together with its model
  file["models"]["ghost"] = kUrdf;
  report = cb::apply(robot, file, local, {{"model_name"}, {"ghost"}});
  EXPECT_EQ(report["applied"], json({"model:ghost", "model_name"}));
  EXPECT_EQ(report["restart_required"], json({"model_name"}));
  EXPECT_EQ(robot.models["ghost"], kUrdf);
}

TEST_F(ConfigBackupTest, ModelCannotBeRestoredOnRollback)
{
  file["models"]["newmodel"] = kUrdf;
  file["parameters"]["default.velocity_limit_scaling"]["value"] = 5.0;
  const auto report = cb::apply(robot, file, local, {{"default.velocity_limit_scaling"}, {"newmodel"}});
  EXPECT_TRUE(report["rolled_back"].get<bool>());
  ASSERT_EQ(report["rollback_errors"].size(), 1u);
  EXPECT_NE(report["rollback_errors"][0].get<std::string>().find("model:newmodel"), std::string::npos);
}

TEST_F(ConfigBackupTest, PortForwardIsSkipped)
{
  file["port_forward"] = {{"forwards", json::array()}, {"snat", true}, {"share_internet", false}};
  file = roundtrip(file);
  const auto preview = cb::build_preview(robot, file, local);
  EXPECT_FALSE(preview["port_forward"]["selected"].get<bool>());
  const auto report = cb::apply(robot, file, local, {{"gravity"}, {}});
  EXPECT_EQ(report["skipped"][0]["name"], "port_forward");
}

TEST_F(ConfigBackupTest, ReadsWebExport)
{
  const auto web = cb::parse_file(read_fixture("web_export.json").dump());
  EXPECT_EQ(web["source"]["fingerprint"], "fp-web-robot");
  const auto preview = cb::build_preview(robot, web, local);
  for (const auto & p : preview["parameters"]) {
    EXPECT_FALSE(p["changed"].get<bool>()) << p["name"];  // the fake robots hold the same values
  }
}

TEST_F(ConfigBackupTest, PreviewMatchesWebPreview)
{
  const auto web_file = cb::parse_file(read_fixture("web_modified.json").dump());
  const auto expected = read_fixture("web_preview_parameters.json");
  const auto preview = cb::build_preview(robot, web_file, local);
  ASSERT_EQ(preview["parameters"].size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(preview["parameters"][i], expected[i]) << expected[i]["name"];
  }
}

TEST_F(ConfigBackupTest, ExportIsAValidWebFile)
{
  // what the web's validate_file requires of a file, checked on the C++ export
  const json exported = cb::build_export(robot, local);
  for (const auto * key : {"format", "format_version", "exported_at", "source", "parameters", "port_forward",
      "models"})
  {
    EXPECT_TRUE(exported.contains(key)) << key;
  }
  EXPECT_TRUE(exported["source"]["robot_model_name"].is_string());
  // RBY1_CONFIG_EXPORT_OUT=<file>: write it out, to be fed to the web backend's parse_file / build_preview
  if (const char * out = std::getenv("RBY1_CONFIG_EXPORT_OUT")) {
    std::ofstream(out) << exported.dump(2) << "\n";
  }
  for (auto it = exported["parameters"].begin(); it != exported["parameters"].end(); ++it) {
    EXPECT_TRUE(cb::value_matches_type(it.value()["type"].get<int>(), it.value()["value"])) << it.key();
  }
}
