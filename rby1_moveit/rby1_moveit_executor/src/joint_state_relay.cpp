// Republish the driver's joint states (/rby1/joint_overall_states) on /joint_states.
//
// The driver publishes with sensor-data QoS (best effort); move_group and
// robot_state_publisher subscribe reliably, which does not match -- a plain remapping
// would leave them without robot state.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateRelay : public rclcpp::Node {
public:
  JointStateRelay() : rclcpp::Node("rby1_moveit_joint_state_relay") {
    const auto source = declare_parameter("source", "/rby1/joint_overall_states");
    const auto target = declare_parameter("target", "/joint_states");
    if (source == target) throw std::invalid_argument("source and target must differ, or the relay feeds itself");
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(target, 10);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      source, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & msg) { publisher_->publish(msg); });
    RCLCPP_INFO(get_logger(), "relaying %s -> %s", source.c_str(), target.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateRelay>());
  rclcpp::shutdown();
  return 0;
}
