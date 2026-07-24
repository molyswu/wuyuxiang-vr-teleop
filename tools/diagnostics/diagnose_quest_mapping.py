import time

import numpy as np

from quest_teleop_ros2.oculus_reader import OculusReader
from quest_teleop_ros2.pose_math import (
    relative_pose,
    quest_to_robot_transform,
    xyzrpy_matrix,
)


reader = OculusReader()
base = None
startup = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0)
last_b = False
print("MAPPING_READY", flush=True)
started = time.time()
try:
    while time.time() - started < 25.0:
        transforms, buttons = reader.get_transformations_and_buttons()
        raw = transforms.get("r")
        b = bool(buttons.get("B", False))
        if raw is None:
            print("MAPPING pose=none B", b, flush=True)
            time.sleep(0.25)
            continue
        mapped = quest_to_robot_transform(raw)
        if base is None:
            base = mapped.copy()
            print("MAPPING_BASE_CAPTURED_FIRST_POSE", flush=True)
        last_b = b
        target = None if base is None else relative_pose(base, mapped, startup)
        print(
            "MAPPING raw=",
            np.round(np.asarray(raw)[:3, 3], 3).tolist(),
            "mapped=",
            np.round(mapped[:3, 3], 3).tolist(),
            "target=",
            None if target is None else np.round(target[:3, 3], 3).tolist(),
            "B=",
            b,
            flush=True,
        )
        time.sleep(0.25)
finally:
    reader.stop()
