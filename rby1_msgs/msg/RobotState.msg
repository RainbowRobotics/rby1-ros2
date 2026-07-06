# Control Manager State Constants
int32 STATE_NONE = 0
int32 STATE_IDLE = 1
int32 STATE_ENABLE = 2
int32 STATE_EXECUTING = 3
int32 STATE_MAJOR_FAULT = 4
int32 STATE_MINOR_FAULT = 5

int32 control_manager_state

# Joint brake engagement status for all components
BrakeState brake_state

# Connection status of tool flanges (index 0: left flange, index 1: right flange)
# True if connected and receiving state updates, False otherwise
bool[] tool_flange_state

# Emergency Stop (EMO) button pressed status (True if pressed/active, False if released)
bool emo_state false

# Calculated Center of Mass (CoM) coordinates: [x, y, z] in meters
float64[3] center_of_mass

# robot stream state
bool robot_stream_state

# Collision status
bool collision

# Robot version
float64 robot_version
 
