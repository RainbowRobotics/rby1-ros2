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

"""Print RESULT lines of validate.py compactly."""
import json, sys
for line in sys.stdin:
    if not line.startswith("RESULT"):
        print(line.rstrip()); continue
    d = json.loads(line[7:]); t = d.pop("test")
    keys = [k for k in d if any(s in k for s in ("error", "moved", "rate", "age", "drift", "distance", "wheel", "rise",
                                                  "overshoot", "success", "hardware", "status", "->"))]
    print(t, json.dumps({k: d[k] for k in keys} if keys else d, default=str)[:700])
