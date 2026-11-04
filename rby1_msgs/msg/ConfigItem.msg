# One parameter of a robot configuration file in an import preview (rby1-config format, as in the web UI).

string name
# Parameter type code (rb/api/parameter.proto): 0 int, 1 double, 2 string, 3 array3, 4 array6, 5 array7.
int32 type
# Values as JSON text; current_json is "null" when this core has no such parameter.
string current_json
string incoming_json
bool changed
# normal, robot_specific, identity (never imported), unknown (not a parameter of this core), type_mismatch.
string category
bool selectable
# Whether an import without an explicit parameter list applies it.
bool selected
# Read by the core only at boot.
bool restart_required
string note
