"""Internal safe home pose captured from the user's confirmed safe posture."""

import math

# Stored internally; do not expose this through launch defaults or logs.
SAFE_HOME_Q = tuple(math.radians(value / 1000.0) for value in (
    0.0, -1715.0, 2368.0, 2516.0, 22708.0, -1371.0,
))
