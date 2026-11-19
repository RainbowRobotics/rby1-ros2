#include <string>

namespace rby1_ros2{
    enum ControlState{
        NONE,
        IDLE,
        ENABLE,
        EXECUTING,
        MAJOR_FAULT,
        MINOR_FAULT
    };

    struct RobotState{
        bool is_connected = false;
        bool is_power_on = false;
        bool is_servo_on = false;
        bool is_ready = false;
        ControlState state = IDLE;
        bool collision = false;
    };

    // Everything driver_parameters.yaml sets, with the yaml's defaults. init_parameter()
    // declares each key and fills this.
    struct DriverParameters{
        std::string robot_ip{"127.0.0.1:50051"};
        std::string model{"m"};
        double stream_command_hz{30.0};   // how often cmd_vel is expected
        // Written to the robot when the driver connects (SDK SetParameter; the robot
        // keeps them in its database): the command filters' cutoff frequencies (Hz) and
        // the default velocity and acceleration limits. At the robot's own defaults.
        struct Robot{
            double joint_position_command_cutoff_frequency{15.0};
            double cartesian_command_cutoff_frequency{15.0};
            double default_acceleration_limit_scaling{1.0};
            double default_velocity_limit_scaling{1.0};
            double default_linear_acceleration_limit{20.0};   // m/s^2
            double default_angular_acceleration_limit{10.0};  // rad/s^2
        } robot;
        double se2_minimum_time{1.0};               // s, cmd_vel interpolation ramp
        double se2_linear_acceleration_limit{0.5};  // m/s^2
        double se2_angular_acceleration_limit{0.5}; // rad/s^2
        double collision_threshold{0.02};           // m
        bool publish_battery_state{true};
        bool publish_tool_flange_state{true};
        // Enable the control manager in unlimited mode (joint limits not checked at
        // enable), also when a plain enable is asked for.
        bool control_manager_unlimited{false};
        // The simulator: it has one power device (named by ".*"), the robot has "48v".
        bool is_simulator() const {
            return robot_ip == "127.0.0.1:50051" || robot_ip == "localhost:50051";
        }
    };
}
 
