// Print the robot's model, model version and software versions as one JSON line.
//   rby1_probe <address> [timeout_ms]
// Used by rby1_bringup to select the URDF without the rby1-sdk Python package.

#include <iostream>
#include <string>

#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"

namespace
{
std::string escape(const std::string & value)
{
  std::string out;
  for (const char c : value) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: rby1_probe <address> [timeout_ms]" << std::endl;
    return 2;
  }
  const std::string address = argv[1];
  const int timeout_ms = argc > 2 ? std::stoi(argv[2]) : 3000;
  try {
    // The RobotInfo service is the same for every model; the A template is enough to read it.
    auto robot = rb::Robot<rb::y1_model::A>::Create(address);
    if (!robot || !robot->Connect(1, timeout_ms)) {
      std::cerr << "cannot connect to " << address << std::endl;
      return 1;
    }
    const auto info = robot->GetRobotInfo();
    std::cout << "{\"model_name\": \"" << escape(info.robot_model_name) << "\", \"model_version\": \""
              << escape(info.robot_model_version) << "\", \"robot_version\": \"" << escape(info.robot_version)
              << "\", \"sdk_version\": \"" << escape(info.sdk_version) << "\", \"dof\": " << info.degree_of_freedom
              << "}" << std::endl;
    robot->Disconnect();
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
