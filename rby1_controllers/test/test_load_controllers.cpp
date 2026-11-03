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

// Each controller is found and instantiated through pluginlib, as the controller manager does.
#include <gmock/gmock.h>

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(LoadControllers, AllPluginsLoad)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  for (const std::string type : {"rby1_controllers/JointStreamController",
      "rby1_controllers/GravityCompensationController", "rby1_controllers/BaseVelocityController",
      "rby1_controllers/TaskSpaceController"})
  {
    std::shared_ptr<controller_interface::ControllerInterface> controller;
    EXPECT_NO_THROW(controller = loader.createSharedInstance(type)) << type;
    EXPECT_NE(controller, nullptr) << type;
  }
}
