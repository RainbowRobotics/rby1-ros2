# Isaac ROS gear assembly on RB-Y1

The Isaac Lab task `IsaacContrib-Deploy-GearAssembly-UR10e-2F140-ROS-Inference` is deployed with
NVIDIA Isaac ROS (`isaac_ros_manipulation_dnn_policy`, release 4.6). This driver provides the robot
side those nodes expect, so they run unmodified with parameters and remaps only.

## What the Isaac ROS nodes need

| Isaac ROS node | Needs from the robot | RB-Y1 driver |
|---|---|---|
| `observation_encoder_node` | `sensor_msgs/JointState` on `joint_state` with fresh stamps (`joint_state_age_threshold_ms`, default 10) | `/joint_states` from `joint_state_broadcaster` at `update_rate` (100 Hz) |
| `action_decoder_node` (streaming mode: `joint_prefix` and `controller_name` set) | a controller taking `sensor_msgs/JointState` on `/<controller_name>/joint_commands` | `right_arm_impedance_controller` (`JointStreamController`, impedance) |
| `action_decoder_node` (legacy UR mode) | `target_joint_state` remapped to `/target_joint_positions` | remap to `/right_arm_impedance_controller/joint_commands` |
| orchestrator | `/controller_manager/switch_controller` between trajectory and impedance controllers | `right_arm_controller` <-> `right_arm_impedance_controller` |
| orchestrator / pick | `control_msgs/action/GripperCommand` | `/right_gripper_controller/gripper_cmd` |

`rby1_bringup/test/test_isaac_ros_contract.py` checks this contract against the fake robot.

## Running

1. Driver (UPC):
   ```bash
   ros2 launch rby1_bringup rby1_control.launch.py gripper:=true \
     activate_controllers:="right_arm_controller right_gripper_controller"
   ```
2. Isaac ROS (in the Isaac ROS container, same `ROS_DOMAIN_ID`, clocks synchronized with chrony):
   ```bash
   ros2 launch isaac_ros_manipulation_dnn_policy inference_flexiv.launch.py \
     controller_name:=right_arm_impedance_controller \
     input_joint_states_topic:=/joint_states \
     input_goal_pose_topic:=/gear_assembly/goal_pose \
     checkpoint:=<rby1 checkpoint>.pt
   ```
   `inference_flexiv.launch.py` is the launch file that already supports streaming mode; its
   `joint_prefix` must be empty for RB-Y1 joint names. If the inference runs on another computer,
   check the stamp age it reports; raise `joint_state_age_threshold_ms` if clocks are not synchronized.
3. Switch to the impedance controller when the policy should take over:
   ```bash
   ros2 control switch_controllers --deactivate right_arm_controller --activate right_arm_impedance_controller
   ```

## The RB-Y1 policy

The UR10e checkpoint cannot drive RB-Y1. An RB-Y1 gear-assembly task has to be trained in Isaac Lab
with:
- `arm_joint_names` = `right_arm_0` ... `right_arm_6`, `action_space` 7, observation
  `[q (7), qd (7), shaft position (3), shaft quaternion xyzw (4)]`;
- the env.yaml keys the inference node reads: `obs_order`, `arm_joint_names`,
  `policy_action_space: joint`, `action_scale_joint_space`, `fixed_asset_init_pos_center`,
  `fixed_asset_init_pos_range`, `fixed_asset_init_orn_deg`, `fixed_asset_init_orn_deg_range`;
- an actuator model of the RB-Y1 joint impedance controller: the robot realizes about 1.3-1.45x the
  commanded stiffness (1.06x on the wrist roll), the reference passes the core's trapezoidal motion
  generator, and the commanded gains are the `JointStreamController` parameters
  (`stiffness` 900/900/900/500/500/500/1000 N·m/rad, `damping_ratio` 1.0,
  `torque_limit` 70/70/70/40/10/10/8 N·m by default).

## Not covered here

Perception (FoundationPose, camera calibration), cuMotion (needs an RB-Y1 XRDF) and the Isaac ROS
orchestrator, whose `robot_type` only knows UR and Flexiv. Running the NVIDIA nodes themselves
requires the Isaac ROS development container (CUDA, `isaac_ros_common`).
