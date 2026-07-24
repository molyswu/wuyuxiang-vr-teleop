# VR LeRobot Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** Add an independent joint-only VR recorder under piper-scripts that writes LeRobot episodes from existing ROS2 topics without modifying the original Piper LeRobot project or controlling the arm.

**Architecture:** vr_record_piper.py subscribes to /joint_command for action and /piper_measured_joint_state for observation, samples synchronized latest values at fixed FPS, and writes observation.state, action, and task through LeRobotDataset. record_vr_piper.sh prepares the vt/ROS2/LeRobot environment and launches the recorder; it never starts Piper SDK, CAN, the VR client, or cameras.

**Tech Stack:** Python 3.10, ROS2 Humble rclpy, sensor_msgs/msg/JointState, LeRobot LeRobotDataset, pytest, Bash.

## Global Constraints

- Reference only: /home/mc509/Workspace/VLA/Piper/lerobot-piper/src/lerobot/scripts/lerobot_record.py.
- Create only piper-scripts/vr_record_piper.py, piper-scripts/record_vr_piper.sh, and tests under piper-scripts/tests/.
- Do not modify Piper/lerobot-piper, quest/lerobot, ROS1/ROS2 source, URDF, Piper SDK, CAN configuration, APK, or EvoDepth.
- Do not import or instantiate piper_sdk, PiperMotorsBus, or any LeRobot robot/teleoperator class.
- Do not start cameras, WebRTC, vr_piper_test.sh, piper_daemon_start.sh, piper_daemon_stop.sh, ip link, sudo, or adb.
- Consume /joint_command and /piper_measured_joint_state as sensor_msgs/msg/JointState.
- Record no images; use seven-element vectors ordered [joint_1..joint_6, gripper].
- Missing gripper values become 0.0; positions shorter than six are rejected.
- Write a frame only after both fresh observation and action messages are available.

---

### Task 1: Conversion helpers and data schema

Files:
- Create: /home/mc509/Workspace/VLA/quest/piper-scripts/vr_record_piper.py
- Create: /home/mc509/Workspace/VLA/quest/piper-scripts/tests/test_vr_record_piper.py

Interfaces:
- joint_state_to_vector(position) -> list[float]
- sample_is_fresh(observation_time, action_time, now, max_age_s) -> bool
- build_frame(observation, action, task) -> dict
- DATASET_FEATURES with float32 seven-element observation.state and action features.

- [ ] Write tests first.

~~~python
def test_joint_state_to_vector_keeps_gripper():
    assert joint_state_to_vector([1, 2, 3, 4, 5, 6, 0.25]) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.25]

def test_joint_state_to_vector_fills_missing_gripper():
    assert joint_state_to_vector([1, 2, 3, 4, 5, 6])[-1] == 0.0

def test_joint_state_to_vector_rejects_short_input():
    with pytest.raises(ValueError):
        joint_state_to_vector([1, 2, 3, 4, 5])

def test_sample_is_fresh_requires_two_recent_messages():
    assert sample_is_fresh(9.8, 9.9, 10.0, 0.25)
    assert not sample_is_fresh(None, 9.9, 10.0, 0.25)
    assert not sample_is_fresh(9.0, 9.9, 10.0, 0.25)

def test_build_frame_has_observation_action_task():
    assert build_frame([1] * 7, [2] * 7, "pick") == {
        "observation.state": [1.0] * 7,
        "action": [2.0] * 7,
        "task": "pick",
    }
~~~

- [ ] Run cd /home/mc509/Workspace/VLA/quest && python -m pytest piper-scripts/tests/test_vr_record_piper.py -q. Expected: failure because helpers are absent.
- [ ] Implement the helpers: reject fewer than six positions, convert to float, add zero gripper when needed, require both fresh timestamps, and return the three exact frame keys.
- [ ] Run the same command; expected GREEN with five passing tests.

### Task 2: Read-only ROS2 subscriber and episode loop

Files:
- Modify: /home/mc509/Workspace/VLA/quest/piper-scripts/vr_record_piper.py
- Modify: /home/mc509/Workspace/VLA/quest/piper-scripts/tests/test_vr_record_piper.py

Interfaces:
- VrJointRecorder(Node) subscribes to joint_command and piper_measured_joint_state.
- command_callback(message) updates only the latest action vector/time.
- measured_callback(message) updates only the latest observation vector/time.
- latest_sample(now) -> tuple[list[float], list[float]] | None.
- record_episodes(args) -> None.

- [ ] Add failing tests for a RecorderState that returns None until both fresh values exist and preserves the last valid sample after a short message.
- [ ] Run the focused pytest command; expected RED because RecorderState is absent.
- [ ] Implement RecorderState and callbacks. Catch ValueError, log a warning, and preserve the previous valid sample. Create only two ROS2 subscriptions; create no publishers, Piper SDK objects, or hardware calls.
- [ ] Implement record_episodes with rclpy.init, rclpy.spin_once(node, timeout_sec=0.0), time.perf_counter, and fixed-FPS sampling. Add only fresh frames, call save_episode after each episode, finalize in finally, and catch KeyboardInterrupt without stopping the daemon.
- [ ] Run all recorder unit tests; expected GREEN.

### Task 3: LeRobotDataset configuration and CLI

Files:
- Modify: /home/mc509/Workspace/VLA/quest/piper-scripts/vr_record_piper.py
- Modify: /home/mc509/Workspace/VLA/quest/piper-scripts/tests/test_vr_record_piper.py

Interfaces:
- Positional CLI: dataset_name, task, num_episodes.
- Options: --root, --fps default 30, --episode-time-s default 60, --max-age-s default 0.25, --resume.
- Repo id: HF_USER environment value or current user plus dataset_name.
- Root default: /home/mc509/Workspace/VLA/quest/datasets/<dataset_name>.
- Robot metadata: piper_vr; use_videos=False.

- [ ] Add failing tests for CLI defaults and exactly two feature keys.
- [ ] Run pytest; expected RED because parser/constants are absent.
- [ ] Implement:

~~~python
JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "gripper"]
DATASET_FEATURES = {
    "observation.state": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
    "action": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
}
~~~

Create new data with LeRobotDataset.create using repo_id, fps, root, robot_type="piper_vr", features=DATASET_FEATURES, and use_videos=False. For resume, load the existing dataset and validate FPS and feature keys.
- [ ] Run pytest; expected GREEN.
- [ ] Run source quest_ros2_env.sh; export PYTHONPATH="$PWD/lerobot/src:$PYTHONPATH"; python piper-scripts/vr_record_piper.py --help. Expected successful help output with no Piper SDK/CAN access.

### Task 4: Shell launcher and integration verification

Files:
- Create: /home/mc509/Workspace/VLA/quest/piper-scripts/record_vr_piper.sh
- Modify: /home/mc509/Workspace/VLA/quest/piper-scripts/tests/test_vr_record_piper.py

Interfaces:
- Run from Quest root as ./piper-scripts/record_vr_piper.sh dataset_name "task description" 5.
- Launcher sources quest_ros2_env.sh, prepends quest/lerobot/src to PYTHONPATH, and executes vr_record_piper.py.
- Launcher never starts VR, Piper daemon, CAN, ADB, or cameras.

- [ ] Add a static test asserting the launcher contains vr_record_piper.py and none of piper_daemon_start.sh, piper_daemon_stop.sh, ip link, sudo, or adb.
- [ ] Run pytest; expected RED because the launcher is absent.
- [ ] Implement:

~~~bash
#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR=$(dirname "$0")
QUEST_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
source "$QUEST_ROOT/quest_ros2_env.sh"
export PYTHONPATH="$QUEST_ROOT/lerobot/src:$PYTHONPATH"
exec python "$SCRIPT_DIR/vr_record_piper.py" "$@"
~~~

Topic checking remains in Python.
- [ ] Run bash -n piper-scripts/record_vr_piper.sh and the full recorder pytest file; expected all pass.
- [ ] Run a recorder-only no-hardware smoke test; with no ROS2 topics it must report a missing-topic error and must not start Piper/CAN. Then start the existing VR control separately and run one short recorder episode.
- [ ] Verify with git status that neither Piper/lerobot-piper nor quest/lerobot received new changes; only pre-existing user changes may remain.

## Self-review

- Tasks cover the approved design, including schema, synchronization, CLI, launcher, and verification.
- No task modifies either original LeRobot project.
- No task creates a Piper SDK/CAN connection.
- Data schema, ROS topics, sampling policy, CLI, and launcher paths are consistent.
- Camera/WebRTC/EvoDepth and VR control startup remain outside recorder scope.

