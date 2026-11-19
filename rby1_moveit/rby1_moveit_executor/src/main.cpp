// rby1_moveit_executor: MoveIt plans, the RB-Y1 driver executes. See executor.hpp.
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "rby1_moveit_executor/executor.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  std::shared_ptr<rby1_moveit_executor::MoveItExecutor> node;
  try {
    node = std::make_shared<rby1_moveit_executor::MoveItExecutor>();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("rby1_target_executor"), "rby1_moveit_executor stopped: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });
  try {
    node->run();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "rby1_moveit_executor stopped: %s", error.what());
    code = 1;
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return code;
}
