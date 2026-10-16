"""Print RESULT lines of validate.py compactly."""
import json, sys
for line in sys.stdin:
    if not line.startswith("RESULT"):
        print(line.rstrip()); continue
    d = json.loads(line[7:]); t = d.pop("test")
    keys = [k for k in d if any(s in k for s in ("error", "moved", "rate", "age", "drift", "distance", "wheel", "rise",
                                                  "overshoot", "success", "hardware", "status", "->"))]
    print(t, json.dumps({k: d[k] for k in keys} if keys else d, default=str)[:700])
