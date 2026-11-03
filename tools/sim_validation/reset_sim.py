# Copyright 2026 Rainbow Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Cancel any control and clear control-manager faults on the simulator core (no driver running)."""
import sys
import time

import rby1_sdk as sdk
r = sdk.create_robot_a(sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1:50051")
assert r.connect()
r.cancel_control()
time.sleep(0.3)
cm = r.get_control_manager_state()
if cm.state in (sdk.ControlManagerState.State.MinorFault, sdk.ControlManagerState.State.MajorFault):
    print("reset fault", r.reset_fault_control_manager())
# Enable only a powered, servoed robot (enabling first and powering later faults the control
# manager); otherwise leave it to the driver's auto_enable.
if r.is_power_on(".*") and r.is_servo_on(".*") and \
        r.get_control_manager_state().state != sdk.ControlManagerState.State.Enabled:
    print("enable", r.enable_control_manager())
print("cm", r.get_control_manager_state().state, r.get_control_manager_state().control_state)
