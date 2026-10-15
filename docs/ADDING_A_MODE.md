# Adding an rby1-sdk controller

1. **Command type** (`rby1_hardware_interface/command_types.hpp`): add an SDK-free struct for the
   command and add it to `ComponentCommand`.
2. **Mode** (`src/command_modes.cpp`): subclass `CommandMode` (or `GpioMode` / `CartesianGpioMode`)
   - `name()`, `required_interfaces()` / `optional_interfaces()` or `matches()`,
   - `on_activate()` to seed command values,
   - `condition()` for the real-time guards (no allocation),
   - `build()` to fill the command struct,
   and create it in `make_modes()` for the components it supports.
3. **SDK translation** (`src/sdk_robot_backend.cpp`): add `InnerOf<YourCommand>` and a `fill()`
   overload; `with_component()` wraps it into the component builders that accept it.
4. **Description**: declare the interfaces (joint interfaces or a `<gpio>` with `kind` and `component`)
   in `rby1.ros2_control.xacro`.
5. **Controller**: reuse a standard controller or `TaskSpaceController`, or add one to
   `rby1_controllers`; add it to `rby1_bringup/config`.
6. **Tests**: a selection/guard/build test in `test/test_command_modes.cpp` and, if the fake backend
   should simulate it, a case in `FakeRobotBackend::send()`.

The system interface, the stream thread and the mode switching need no changes.
