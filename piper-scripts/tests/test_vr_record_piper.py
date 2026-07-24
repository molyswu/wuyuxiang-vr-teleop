import sys
import getpass
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

import vr_record_piper  # noqa: E402
from vr_record_piper import (  # noqa: E402
    RecorderState,
    VrJointRecorder,
    build_frame,
    joint_state_to_vector,
    record_episodes,
    sample_is_fresh,
    wait_for_required_topics,
)


def test_cli_defaults_and_repo_root(monkeypatch):
    monkeypatch.delenv("HF_USER", raising=False)
    monkeypatch.setattr(sys, "argv", ["vr_record_piper.py", "demo", "pick", "3"])

    args = vr_record_piper.parse_args()

    assert args.fps == 30
    assert args.episode_time_s == 60.0
    assert args.max_age_s == 0.25
    assert args.topic_timeout_s == 5.0
    assert args.resume is False
    assert args.repo_id == f"{getpass.getuser()}/demo"
    assert args.root == Path(vr_record_piper.DEFAULT_ROOT_BASE) / "demo"


def test_cli_honors_hf_user(monkeypatch):
    monkeypatch.setenv("HF_USER", "dataset-owner")
    monkeypatch.setattr(sys, "argv", ["vr_record_piper.py", "demo", "pick", "1"])

    args = vr_record_piper.parse_args()

    assert args.repo_id == "dataset-owner/demo"


def test_dataset_features_are_exactly_joint_only_keys():
    assert list(vr_record_piper.DATASET_FEATURES) == [
        "observation.state",
        "action",
    ]
    assert vr_record_piper.DATASET_FEATURES["observation.state"] == {
        "dtype": "float32",
        "shape": (7,),
        "names": vr_record_piper.JOINT_NAMES,
    }
    assert vr_record_piper.DATASET_FEATURES["action"] == {
        "dtype": "float32",
        "shape": (7,),
        "names": vr_record_piper.JOINT_NAMES,
    }


def test_new_dataset_passes_piper_configuration(monkeypatch):
    calls = []

    class FakeDataset:
        @classmethod
        def create(cls, **kwargs):
            calls.append(kwargs)
            return cls()

    monkeypatch.setitem(
        sys.modules,
        "lerobot.datasets.lerobot_dataset",
        SimpleNamespace(LeRobotDataset=FakeDataset),
    )
    args = SimpleNamespace(
        repo_id="user/demo",
        fps=30,
        root=Path("/tmp/demo"),
        resume=False,
    )

    vr_record_piper._make_dataset(args)

    assert calls == [{
        "repo_id": "user/demo",
        "fps": 30,
        "root": Path("/tmp/demo"),
        "robot_type": "piper_vr",
        "features": vr_record_piper.DATASET_FEATURES,
        "use_videos": False,
    }]


def test_resume_dataset_validates_fps_and_feature_keys(monkeypatch):
    class FakeDataset:
        def __init__(self, **kwargs):
            self.meta = SimpleNamespace(fps=30, features=vr_record_piper.DATASET_FEATURES)

    monkeypatch.setitem(
        sys.modules,
        "lerobot.datasets.lerobot_dataset",
        SimpleNamespace(LeRobotDataset=FakeDataset),
    )
    args = SimpleNamespace(
        repo_id="user/demo",
        fps=30,
        root=Path("/tmp/demo"),
        resume=True,
    )

    dataset = vr_record_piper._make_dataset(args)

    assert isinstance(dataset, FakeDataset)


def test_resume_dataset_rejects_fps_mismatch(monkeypatch):
    class FakeDataset:
        def __init__(self, **kwargs):
            self.meta = SimpleNamespace(fps=60, features=vr_record_piper.DATASET_FEATURES)

    monkeypatch.setitem(
        sys.modules,
        "lerobot.datasets.lerobot_dataset",
        SimpleNamespace(LeRobotDataset=FakeDataset),
    )
    args = SimpleNamespace(
        repo_id="user/demo",
        fps=30,
        root=Path("/tmp/demo"),
        resume=True,
    )

    with pytest.raises(ValueError, match="does not match requested FPS"):
        vr_record_piper._make_dataset(args)


def test_resume_dataset_rejects_feature_key_mismatch(monkeypatch):
    class FakeDataset:
        def __init__(self, **kwargs):
            self.meta = SimpleNamespace(
                fps=30,
                features={"observation.state": vr_record_piper.DATASET_FEATURES["observation.state"]},
            )

    monkeypatch.setitem(
        sys.modules,
        "lerobot.datasets.lerobot_dataset",
        SimpleNamespace(LeRobotDataset=FakeDataset),
    )
    args = SimpleNamespace(
        repo_id="user/demo",
        fps=30,
        root=Path("/tmp/demo"),
        resume=True,
    )

    with pytest.raises(ValueError, match="feature keys do not match"):
        vr_record_piper._make_dataset(args)


def test_recorder_state_returns_none_until_both_values_are_fresh():
    state = RecorderState(max_age_s=0.25)

    assert state.latest_sample(10.0) is None
    state.update_action([1] * 7, 9.9)
    assert state.latest_sample(10.0) is None
    state.update_observation([2] * 7, 9.9)
    assert state.latest_sample(10.0) == ([2.0] * 7, [1.0] * 7)
    assert state.latest_sample(10.14) == ([2.0] * 7, [1.0] * 7)
    assert state.latest_sample(10.16) is None


def test_recorder_state_preserves_last_valid_sample_after_invalid_message():
    state = RecorderState(max_age_s=0.25)
    state.update_action([1] * 7, 9.9)
    state.update_observation([2] * 7, 9.9)

    with pytest.raises(ValueError):
        state.update_action(["not-a-number"] * 7, 10.0)

    assert state.latest_sample(10.1) == ([2.0] * 7, [1.0] * 7)


def test_joint_state_to_vector_keeps_gripper():
    assert joint_state_to_vector([1, 2, 3, 4, 5, 6, 0.25]) == [
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        0.25,
    ]


def test_joint_state_to_vector_fills_missing_gripper():
    assert joint_state_to_vector([1, 2, 3, 4, 5, 6])[-1] == 0.0


def test_joint_state_to_vector_rejects_short_input():
    with pytest.raises(ValueError):
        joint_state_to_vector([1, 2, 3, 4, 5])


def test_sample_is_fresh_requires_two_recent_messages():
    assert sample_is_fresh(9.8, 9.9, 10.0, 0.25)
    assert not sample_is_fresh(None, 9.9, 10.0, 0.25)
    assert not sample_is_fresh(9.0, 9.9, 10.0, 0.25)


def test_sample_is_fresh_rejects_future_timestamp():
    assert not sample_is_fresh(10.1, 9.9, 10.0, 0.25)


def test_command_callback_updates_state_and_warns_without_discarding_valid_sample():
    recorder = object.__new__(VrJointRecorder)
    recorder.state = RecorderState()
    recorder._clock = lambda: 10.0
    warnings = []
    recorder._warn = warnings.append

    recorder.command_callback(SimpleNamespace(position=[1] * 7))
    recorder.command_callback(SimpleNamespace(position=[1] * 5))
    recorder.command_callback(SimpleNamespace(position=["bad"] * 7))

    assert recorder.state.action == [1.0] * 7
    assert recorder.state.action_time == 10.0
    assert len(warnings) == 2


def test_measured_callback_updates_state_and_warns_without_discarding_valid_sample():
    recorder = object.__new__(VrJointRecorder)
    recorder.state = RecorderState()
    recorder._clock = lambda: 11.0
    warnings = []
    recorder._warn = warnings.append

    recorder.measured_callback(SimpleNamespace(position=[2] * 7))
    recorder.measured_callback(SimpleNamespace(position=[2] * 5))
    recorder.measured_callback(SimpleNamespace(position=["bad"] * 7))

    assert recorder.state.observation == [2.0] * 7
    assert recorder.state.observation_time == 11.0
    assert len(warnings) == 2


def test_record_episodes_shuts_down_when_node_construction_fails(monkeypatch):
    class FakeRclpy:
        def __init__(self):
            self.shutdown_called = False

        def init(self):
            pass

        def shutdown(self):
            self.shutdown_called = True

    fake_rclpy = FakeRclpy()

    def fail_to_construct(*_args, **_kwargs):
        raise RuntimeError("node construction failed")

    monkeypatch.setattr(vr_record_piper, "rclpy", fake_rclpy)
    monkeypatch.setattr(vr_record_piper, "VrJointRecorder", fail_to_construct)

    with pytest.raises(RuntimeError, match="node construction failed"):
        record_episodes(SimpleNamespace())

    assert fake_rclpy.shutdown_called


def test_topic_preflight_succeeds_when_both_topics_are_advertised(monkeypatch):
    class FakeNode:
        def get_topic_names_and_types(self):
            return [("/joint_command", ["sensor_msgs/msg/JointState"]),
                    ("/piper_measured_joint_state", ["sensor_msgs/msg/JointState"])]

    spins = []
    monkeypatch.setattr(vr_record_piper, "rclpy", SimpleNamespace(spin_once=lambda *_args, **_kwargs: spins.append(True)))
    wait_for_required_topics(FakeNode(), 0.0, clock=lambda: 0.0, sleep=lambda _: None)
    assert spins == []


def test_topic_preflight_reports_missing_topics_after_timeout(monkeypatch):
    class FakeNode:
        def get_topic_names_and_types(self):
            return [("/joint_command", ["sensor_msgs/msg/JointState"])]

    now = [0.0]
    monkeypatch.setattr(vr_record_piper, "rclpy", SimpleNamespace(spin_once=lambda *_args, **_kwargs: None))

    def clock():
        value = now[0]
        now[0] += 0.1
        return value

    with pytest.raises(RuntimeError, match="/piper_measured_joint_state"):
        wait_for_required_topics(FakeNode(), 0.15, clock=clock, sleep=lambda _: None)


def test_topic_preflight_reports_wrong_message_type_after_timeout(monkeypatch):
    class FakeNode:
        def get_topic_names_and_types(self):
            return [
                ("/joint_command", ["std_msgs/msg/Float32"]),
                ("/piper_measured_joint_state", ["sensor_msgs/msg/JointState"]),
            ]

    now = [0.0]
    monkeypatch.setattr(vr_record_piper, "rclpy", SimpleNamespace(spin_once=lambda *_args, **_kwargs: None))

    def clock():
        value = now[0]
        now[0] += 0.1
        return value

    with pytest.raises(RuntimeError, match="wrong type.*joint_command"):
        wait_for_required_topics(FakeNode(), 0.15, clock=clock, sleep=lambda _: None)


def test_topic_preflight_rejects_mixed_message_types_after_timeout(monkeypatch):
    class FakeNode:
        def get_topic_names_and_types(self):
            return [
                ("/joint_command", [
                    "sensor_msgs/msg/JointState",
                    "std_msgs/msg/Float32",
                ]),
                ("/piper_measured_joint_state", ["sensor_msgs/msg/JointState"]),
            ]

    now = [0.0]
    monkeypatch.setattr(vr_record_piper, "rclpy", SimpleNamespace(spin_once=lambda *_args, **_kwargs: None))

    def clock():
        value = now[0]
        now[0] += 0.1
        return value

    with pytest.raises(RuntimeError, match=r"wrong type.*joint_command.*Float32"):
        wait_for_required_topics(FakeNode(), 0.15, clock=clock, sleep=lambda _: None)


def test_launcher_is_safe_and_wires_ros_environment():
    launcher = Path(__file__).parents[1] / "record_vr_piper.sh"
    text = launcher.read_text()

    assert text.startswith("#!/usr/bin/env bash")
    assert "set -Eeuo pipefail" in text
    assert 'source "$QUEST_ROOT/quest_ros2_env.sh"' in text
    assert 'PYTHONPATH="$QUEST_ROOT/lerobot/src${PYTHONPATH:+:$PYTHONPATH}"' in text
    assert 'exec python "$SCRIPT_DIR/vr_record_piper.py" "$@"' in text
    lowered = text.lower()
    for forbidden in ("sudo", "ip link", "piper_daemon", "adb", "ros2 launch", "evo-depth", "camera"):
        assert forbidden not in lowered


def test_build_frame_has_observation_action_task():
    assert build_frame([1] * 7, [2] * 7, "pick") == {
        "observation.state": [1.0] * 7,
        "action": [2.0] * 7,
        "task": "pick",
    }
