import time

import numpy as np

from quest_teleop_ros2.oculus_reader import OculusReader


reader = OculusReader()
last = None
print("RAW_READY", flush=True)
started = time.time()
try:
    while time.time() - started < 20.0:
        transforms, buttons = reader.get_transformations_and_buttons()
        matrix = transforms.get("r")
        position = None if matrix is None else np.asarray(matrix, dtype=float)[:3, 3]
        delta = None
        if position is not None and last is not None:
            delta = float(np.linalg.norm(position - last))
        print(
            "RAW_R",
            None if position is None else np.round(position, 4).tolist(),
            "DELTA",
            delta,
            "A",
            bool(buttons.get("A", False)),
            "B",
            bool(buttons.get("B", False)),
            flush=True,
        )
        last = None if position is None else position.copy()
        time.sleep(0.5)
finally:
    reader.stop()
