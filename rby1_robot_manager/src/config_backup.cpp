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

#include "rby1_robot_manager/config_backup.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <map>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rby1_robot_manager
{
namespace config_backup
{
namespace
{

// Never applied: a different value makes the core factory-reset every parameter at boot.
const std::set<std::string> kIdentityParameters = {"robot_model_name"};
// Values that belong to one robot (battery pack, mounting, network identity, URDF).
const std::set<std::string> kRobotSpecificParameters = {
  "battery_config", "gravity", "hotspot.ssid", "hotspot.password", "model_name"};
// Read only at boot by app_main.
const std::set<std::string> kRestartRequiredParameters = {
  "model_name", "boot.power_on_target", "major_fault_power_off_target"};

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

bool is_number(const json & v)
{
  if (!v.is_number()) {
    return false;
  }
  return !v.is_number_float() || std::isfinite(v.get<double>());
}

size_t array_length(int type)
{
  switch (type) {
    case kArray3: return 3;
    case kArray6: return 6;
    case kArray7: return 7;
    default: return 0;
  }
}

bool is_int(const json & v) {return v.is_number_integer();}

std::string type_text(const json & v) {return v.dump();}

json current_parameters(ConfigRobot & robot)
{
  json current = json::object();
  for (const auto & [name, type] : robot.parameter_list()) {
    json value = nullptr;
    try {
      value = json::parse(robot.get_parameter(name));
    } catch (const std::exception &) {
      value = nullptr;
    }
    current[name] = {{"type", type}, {"value", value}};
  }
  return current;
}

struct Classification
{
  std::string category;
  bool selectable;
  bool selected;
  std::string note;
};

Classification classify(
  const std::string & name, const json & entry, const json & current, bool same_robot,
  const std::set<std::string> & models_available)
{
  if (kIdentityParameters.count(name)) {
    return {"identity", false, false, "never imported"};
  }
  if (!current.contains(name)) {
    return {"unknown", false, false, "not a parameter of this core"};
  }
  const int current_type = current[name]["type"].get<int>();
  if (current_type != entry["type"].get<int>()) {
    return {"type_mismatch", false, false, "type differs (this core: " + std::to_string(current_type) + ")"};
  }
  const bool changed = current[name]["value"] != entry["value"];
  if (name == "model_name" && entry["value"].is_string()) {
    const auto model = entry["value"].get<std::string>();
    if (!model.empty() && !models_available.count(model)) {
      return {"robot_specific", false, false,
        "model '" + model + "' is not in the file (the robot's model directory is not visible over gRPC)"};
    }
  }
  if (kRobotSpecificParameters.count(name)) {
    return {"robot_specific", true, changed && same_robot, ""};
  }
  return {"normal", true, changed, ""};
}

std::string iso_now()
{
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char buffer[40];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &local);
  std::string text(buffer);
  if (text.size() >= 5) {
    text.insert(text.size() - 2, ":");  // +0900 -> +09:00, as Python's isoformat
  }
  return text;
}

std::set<std::string> keys(const json & object)
{
  std::set<std::string> out;
  for (auto it = object.begin(); it != object.end(); ++it) {
    out.insert(it.key());
  }
  return out;
}

}  // namespace

bool is_identity_parameter(const std::string & name) {return kIdentityParameters.count(name) > 0;}
bool is_robot_specific_parameter(const std::string & name) {return kRobotSpecificParameters.count(name) > 0;}
bool is_restart_required_parameter(const std::string & name) {return kRestartRequiredParameters.count(name) > 0;}

bool is_valid_model_name(const std::string & name)
{
  static const std::regex pattern("^[A-Za-z0-9_.-]{1,64}$");
  return std::regex_match(name, pattern);
}

bool value_matches_type(int type, const json & value)
{
  switch (type) {
    case kInt: return is_int(value);
    case kDouble: return is_number(value);
    case kString: return value.is_string();
    case kArray3:
    case kArray6:
    case kArray7:
      return value.is_array() && value.size() == array_length(type) &&
             std::all_of(value.begin(), value.end(), [](const json & x) {return is_number(x);});
    default: return false;
  }
}

// region export

json local_source(ConfigRobot & robot, const std::string & hostname)
{
  const auto id = robot.identity();
  return {
    {"robot_model_name", lower(id.model_name)},
    {"robot_model_version", lower(id.model_version)},
    {"core_version", id.core_version},
    {"sdk_version", id.sdk_version},
    {"hostname", hostname},
    {"fingerprint", nullptr},
    {"exporter", "rby1_robot_manager"},
  };
}

json build_export(ConfigRobot & robot, const json & source, std::vector<std::string> * warnings)
{
  json parameters = json::object();
  for (const auto & [name, type] : robot.parameter_list()) {
    try {
      parameters[name] = {{"type", type}, {"value", json::parse(robot.get_parameter(name))}};
    } catch (const std::exception & e) {
      if (warnings) {
        warnings->push_back("skipping parameter " + name + ": " + e.what());
      }
    }
  }
  return {
    {"format", kFormat},
    {"format_version", kFormatVersion},
    {"exported_at", iso_now()},
    {"source", source},
    {"parameters", parameters},
    {"port_forward", nullptr},
    {"models", json::object()},
  };
}

std::string export_filename(const json & source)
{
  std::string model = "robot";
  if (source.contains("robot_model_name") && source["robot_model_name"].is_string() &&
    !source["robot_model_name"].get<std::string>().empty())
  {
    model = source["robot_model_name"].get<std::string>();
  }
  const std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M%S", &local);
  return "rby1-config-" + model + "-" + buffer + ".json";
}

// endregion

// region validation

json parse_file(const std::string & data)
{
  if (data.size() > kMaxFileBytes) {
    throw ConfigError("file is larger than " + std::to_string(kMaxFileBytes / 1024) + " KiB");
  }
  json obj;
  try {
    obj = json::parse(data);
  } catch (const json::parse_error & e) {
    throw ConfigError(std::string("not a JSON file: ") + e.what());
  }
  return validate_file(obj);
}

json validate_file(const json & obj)
{
  if (!obj.is_object() || !obj.contains("format") || obj["format"] != kFormat) {
    throw ConfigError("not an RB-Y1 config file");
  }
  if (!obj.contains("format_version") || !is_int(obj["format_version"]) || obj["format_version"] != kFormatVersion) {
    const std::string got = obj.contains("format_version") ? type_text(obj["format_version"]) : "None";
    const std::string expected = std::to_string(kFormatVersion);
    throw ConfigError("unsupported format_version " + got + " (expected " + expected + ")");
  }

  const json source = obj.value("source", json());
  if (!source.is_object() || !source.contains("robot_model_name") || !source["robot_model_name"].is_string()) {
    throw ConfigError("source.robot_model_name is missing");
  }

  const json params = obj.value("parameters", json());
  if (!params.is_object()) {
    throw ConfigError("parameters must be an object");
  }
  for (auto it = params.begin(); it != params.end(); ++it) {
    const auto & entry = it.value();
    if (!entry.is_object() || !entry.contains("type") || !is_int(entry["type"]) || !entry.contains("value")) {
      throw ConfigError("parameter " + it.key() + ": expected {type, value}");
    }
    if (!value_matches_type(entry["type"].get<int>(), entry["value"])) {
      throw ConfigError("parameter " + it.key() + ": value does not match type " + entry["type"].dump());
    }
  }

  json models = obj.value("models", json());
  if (models.is_null()) {
    models = json::object();
  }
  if (!models.is_object()) {
    throw ConfigError("models must be an object");
  }
  for (auto it = models.begin(); it != models.end(); ++it) {
    if (!is_valid_model_name(it.key())) {
      throw ConfigError("invalid model name '" + it.key() + "'");
    }
    const auto & text = it.value();
    const bool blank = !text.is_string() ||
      std::all_of(
      text.get_ref<const std::string &>().begin(), text.get_ref<const std::string &>().end(),
      [](unsigned char c) {return std::isspace(c);});
    if (blank) {
      throw ConfigError("model " + it.key() + ": URDF text is empty");
    }
  }

  const json pf = obj.value("port_forward", json());
  if (!pf.is_null()) {
    const bool forwards_ok = !pf.is_object() || !pf.contains("forwards") || pf["forwards"].is_array();
    if (!pf.is_object() || !forwards_ok) {
      throw ConfigError("port_forward must be an object with a forwards list");
    }
  }

  return {
    {"format", kFormat},
    {"format_version", kFormatVersion},
    {"exported_at", obj.value("exported_at", json())},
    {"source", source},
    {"parameters", params},
    {"port_forward", pf},
    {"models", models},
  };
}

void check_same_model(const json & file, const json & local)
{
  const auto file_model = file["source"]["robot_model_name"].get<std::string>();
  if (lower(file_model) != local["robot_model_name"].get<std::string>()) {
    const auto local_model = local["robot_model_name"].get<std::string>();
    throw ConfigError("file is from robot model '" + file_model + "', this robot is '" + local_model + "'");
  }
}

bool is_same_robot(const json & file, const json & local)
{
  const auto & source = file["source"];
  if (!source.contains("fingerprint") || !source["fingerprint"].is_string() ||
    source["fingerprint"].get<std::string>().empty())
  {
    return false;
  }
  return local.contains("fingerprint") && source["fingerprint"] == local["fingerprint"];
}

// endregion

// region preview

json build_preview(ConfigRobot & robot, const json & file, const json & local)
{
  check_same_model(file, local);
  const bool same_robot = is_same_robot(file, local);
  const json current = current_parameters(robot);
  const auto models_available = keys(file["models"]);

  json warnings = json::array();
  if (!same_robot) {
    warnings.push_back(
      "This file comes from another robot (or its origin is unknown): robot-specific values "
      "(battery, gravity, hotspot SSID/password, model) are not selected by default.");
  }
  if (file["source"].value("core_version", json()) != local.value("core_version", json())) {
    warnings.push_back(
      "Core version differs: file " + file["source"].value("core_version", json()).dump() + ", robot " +
      local.value("core_version", json()).dump() + ".");
  }
  if (!file["port_forward"].is_null()) {
    warnings.push_back("The file has port forward settings; import them with the web UI (not reachable over gRPC).");
  }

  json parameters = json::array();
  for (auto it = file["parameters"].begin(); it != file["parameters"].end(); ++it) {  // sorted by name
    const auto & name = it.key();
    const auto & entry = it.value();
    const auto c = classify(name, entry, current, same_robot, models_available);
    const bool has_current = current.contains(name);
    json item = json::object();
    item["name"] = name;
    item["type"] = entry["type"];
    item["current"] = has_current ? current[name]["value"] : json();
    item["incoming"] = entry["value"];
    item["changed"] = !has_current || current[name]["value"] != entry["value"];
    item["category"] = c.category;
    item["selectable"] = c.selectable;
    item["selected"] = c.selected;
    item["restart_required"] = is_restart_required_parameter(name);
    item["note"] = c.note;
    parameters.push_back(item);
  }

  json models = json::array();
  for (auto it = file["models"].begin(); it != file["models"].end(); ++it) {
    models.push_back({{"name", it.key()}, {"status", "unknown"}, {"selected", false}});
  }

  json port_forward = nullptr;
  if (!file["port_forward"].is_null()) {
    port_forward = {
      {"current", nullptr}, {"incoming", file["port_forward"]}, {"changed", true}, {"selected", false},
      {"note", "not reachable over gRPC; use the web UI"}};
  }

  return {
    {"source", file["source"]},
    {"exported_at", file.value("exported_at", json())},
    {"local", local},
    {"same_robot", same_robot},
    {"warnings", warnings},
    {"parameters", parameters},
    {"models", models},
    {"port_forward", port_forward},
  };
}

std::vector<std::string> default_parameters(const json & preview, bool include_robot_specific)
{
  std::vector<std::string> names;
  for (const auto & p : preview["parameters"]) {
    const bool robot_specific = p["category"] == "robot_specific";
    if (p["selected"].get<bool>() ||
      (include_robot_specific && robot_specific && p["selectable"].get<bool>() && p["changed"].get<bool>()))
    {
      names.push_back(p["name"].get<std::string>());
    }
  }
  return names;
}

// endregion

// region apply

namespace
{

void rollback(
  ConfigRobot & robot, const std::vector<std::pair<std::string, json>> & param_backups,
  const std::vector<std::string> & imported_models, json & report)
{
  json errors = json::array();
  for (auto it = param_backups.rbegin(); it != param_backups.rend(); ++it) {
    try {
      if (!robot.set_parameter(it->first, it->second.dump(), true)) {
        errors.push_back(it->first);
      }
    } catch (const std::exception &) {
      errors.push_back(it->first);
    }
  }
  // The previous URDF of an imported model cannot be read back or deleted over gRPC.
  for (auto it = imported_models.rbegin(); it != imported_models.rend(); ++it) {
    errors.push_back("model:" + *it + " (cannot be restored over gRPC)");
  }
  report["rolled_back"] = true;
  report["rollback_errors"] = errors;
  report["applied"] = json::array();
  report["restart_required"] = json::array();
}

template<typename T>
std::vector<T> unique(const std::vector<T> & in)
{
  std::vector<T> out;
  for (const auto & v : in) {
    if (std::find(out.begin(), out.end(), v) == out.end()) {
      out.push_back(v);
    }
  }
  return out;
}

}  // namespace

json apply(ConfigRobot & robot, const json & file, const json & local, const Selection & selection)
{
  check_same_model(file, local);
  const auto selected_params = unique(selection.parameters);
  const auto selected_models = unique(selection.models);

  const json current = current_parameters(robot);
  json report = {
    {"applied", json::array()}, {"failed", json::array()}, {"skipped", json::array()}, {"rolled_back", false},
    {"restart_required", json::array()}, {"port_forward", nullptr}};

  for (const auto & name : selected_models) {
    if (!file["models"].contains(name)) {
      throw ConfigError("model " + name + " is not in the file");
    }
  }

  const std::set<std::string> models_available(selected_models.begin(), selected_models.end());
  std::vector<std::pair<std::string, json>> to_set;
  for (const auto & name : selected_params) {
    if (!file["parameters"].contains(name)) {
      throw ConfigError("parameter " + name + " is not in the file");
    }
    const auto & entry = file["parameters"][name];
    const auto c = classify(name, entry, current, true, models_available);
    if (!c.selectable) {
      report["skipped"].push_back({{"name", name}, {"detail", c.note}});
      continue;
    }
    to_set.emplace_back(name, entry["value"]);
  }
  if (!file["port_forward"].is_null()) {
    report["skipped"].push_back(
      {{"name", "port_forward"}, {"detail", "not reachable over gRPC; use the web UI"}});
  }

  // 1. models
  std::vector<std::string> imported_models;
  for (const auto & name : selected_models) {
    bool ok = false;
    std::string err = "the core rejected the model (joint names differ?)";
    try {
      ok = robot.import_robot_model(name, file["models"][name].get<std::string>());
    } catch (const std::exception & e) {
      ok = false;
      err = e.what();
    }
    if (!ok) {
      report["failed"].push_back({{"name", "model:" + name}, {"detail", err}});
      rollback(robot, {}, imported_models, report);
      return report;
    }
    imported_models.push_back(name);
    report["applied"].push_back("model:" + name);
  }

  // 2. parameters
  std::vector<std::pair<std::string, json>> param_backups;
  for (const auto & [name, value] : to_set) {
    const json previous = current[name]["value"];
    bool ok = false;
    std::string err = "rejected by the core (out of range?)";
    try {
      ok = robot.set_parameter(name, value.dump(), true);
    } catch (const std::exception & e) {
      ok = false;
      err = e.what();
    }
    if (!ok) {
      report["failed"].push_back({{"name", name}, {"detail", err}});
      rollback(robot, param_backups, imported_models, report);
      return report;
    }
    param_backups.emplace_back(name, previous);
    report["applied"].push_back(name);
    if (is_restart_required_parameter(name)) {
      report["restart_required"].push_back(name);
    }
  }
  return report;
}

// endregion

}  // namespace config_backup
}  // namespace rby1_robot_manager
