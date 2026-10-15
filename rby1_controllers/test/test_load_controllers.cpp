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
