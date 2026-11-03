# Tool flange state of one arm (rby1-sdk ToolFlangeState). SI units (REP 103) unless stated.

# Force/torque sensor: force [Fx, Fy, Fz] in N, torque [Tx, Ty, Tz] in N·m.
float64[3] ft_force
float64[3] ft_torque

# Tool flange IMU: angular rate in rad/s, linear acceleration in m/s^2.
float64[3] gyro
float64[3] acceleration

# Switch A state (true when pressed).
bool switch_a

# Output voltage supplied to the tool, raw rby1-sdk value (the SDK does not document the unit).
int32 output_voltage

# Digital inputs and outputs (true = high).
bool digital_input_a
bool digital_input_b
bool digital_output_a
bool digital_output_b
