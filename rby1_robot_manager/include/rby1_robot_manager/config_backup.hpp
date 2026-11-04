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

#pragma once

// Export / import of the robot configuration in the format of the rby1 web UI
// (rby1-web/rby1-backend/config_backup.py, "rby1-config" format_version 1).
//
// The logic follows the web module function by function so a file written by either side is read by the
// other. Everything goes through rby1-sdk gRPC calls, which cannot reach three things the web backend reads
// on the robot PC itself:
//   * the port forward settings: never exported; an imported file's port_forward is reported as skipped;
//   * the custom model directory: models are not exported; models in a file can be imported, but their
//     status on the robot is unknown and a model cannot be restored on rollback;
//   * the NIC fingerprint: the local fingerprint is unknown, so a file is never treated as coming from this
//     same robot and robot-specific values are applied only when selected explicitly.

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

namespace rby1_robot_manager
{
namespace config_backup
{

using json = nlohmann::json;

constexpr const char * kFormat = "rby1-config";
constexpr int kFormatVersion = 1;
constexpr size_t kMaxFileBytes = 1024 * 1024;

// Parameter type codes (rb/api/parameter.proto).
enum ParameterType : int { kInt = 0, kDouble = 1, kString = 2, kArray3 = 3, kArray6 = 4, kArray7 = 5 };

// Invalid file or request (HTTP 400 in the web backend).
class ConfigError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// The robot calls the module needs (the SDK, or a fake in tests).
class ConfigRobot
{
public:
  struct Identity
  {
    std::string model_name;
    std::string model_version;
    std::string core_version;
    std::string sdk_version;
  };

  virtual ~ConfigRobot() = default;
  virtual Identity identity() = 0;
  virtual std::vector<std::pair<std::string, int>> parameter_list() = 0;
  // JSON text of the value; throws on failure.
  virtual std::string get_parameter(const std::string & name) = 0;
  virtual bool set_parameter(const std::string & name, const std::string & json_value, bool write_db) = 0;
  virtual bool import_robot_model(const std::string & name, const std::string & urdf) = 0;
};

bool is_identity_parameter(const std::string & name);
bool is_robot_specific_parameter(const std::string & name);
bool is_restart_required_parameter(const std::string & name);
bool is_valid_model_name(const std::string & name);
bool value_matches_type(int type, const json & value);

// source block of this robot; fingerprint is null (not reachable over gRPC).
json local_source(ConfigRobot & robot, const std::string & hostname);
// Whole file; parameters that cannot be read are skipped and named in `warnings`.
json build_export(ConfigRobot & robot, const json & source, std::vector<std::string> * warnings = nullptr);
std::string export_filename(const json & source);

json parse_file(const std::string & data);
json validate_file(const json & file);
void check_same_model(const json & file, const json & local);
bool is_same_robot(const json & file, const json & local);

// Same keys as the web preview: source, exported_at, local, same_robot, warnings, parameters, models,
// port_forward. Model status is "unknown" (the model directory is not reachable over gRPC).
json build_preview(ConfigRobot & robot, const json & file, const json & local);

struct Selection
{
  std::vector<std::string> parameters;
  std::vector<std::string> models;
};

// Parameters the web preview selects by default (changed, not robot-specific unless the file comes from
// this robot); `include_robot_specific` adds the changed robot-specific values.
std::vector<std::string> default_parameters(const json & preview, bool include_robot_specific);

// Applies `selection`; parameters are all-or-nothing (rolled back on the first failure). Same report keys
// as the web: applied, failed, skipped, rolled_back, restart_required, port_forward (+ rollback_errors).
json apply(ConfigRobot & robot, const json & file, const json & local, const Selection & selection);

}  // namespace config_backup
}  // namespace rby1_robot_manager
