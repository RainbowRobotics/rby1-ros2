# One tool flange, in RobotState (tool_flange_left, tool_flange_right). Its IMU and force/torque
# sensor are on the tool_flange/<side>/imu and /wrench topics in standard types.

# True while the robot receives state updates from this flange (connected)
bool connected

# Supplied output voltage to the tool in volts: 0, 12 or 24, as set with tool_flange_power
int32 output_voltage

# Physical switch A state (True if active/pressed)
bool switch_a
