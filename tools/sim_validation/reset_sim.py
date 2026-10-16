"""Cancel any control and clear control-manager faults on the simulator core (no driver running)."""
import sys, time
import rby1_sdk as sdk
r = sdk.create_robot_a(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:50051")
assert r.connect()
r.cancel_control(); time.sleep(0.3)
cm = r.get_control_manager_state()
if cm.state in (sdk.ControlManagerState.State.MinorFault, sdk.ControlManagerState.State.MajorFault):
    print("reset fault", r.reset_fault_control_manager())
if r.get_control_manager_state().state != sdk.ControlManagerState.State.Enabled:
    print("enable", r.enable_control_manager())
print("cm", r.get_control_manager_state().state, r.get_control_manager_state().control_state)
