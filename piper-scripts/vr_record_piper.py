"""Read-only ROS2 subscriber and episode recorder for VR Piper data."""

import argparse
import getpass
import os
from pathlib import Path
import time

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError:  # Unit tests can exercise the pure state machine without ROS2.
    rclpy = None

    class Node:
        def __init__(self, *_args, **_kwargs):
            self._logger = None

        def get_logger(self):
            return self._logger

    JointState = object


JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "gripper",
]
DATASET_FEATURES = {
    "observation.state": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
    "action": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
}
DEFAULT_ROOT_BASE = Path("/home/mc509/Workspace/VLA/quest/datasets")
REQUIRED_TOPICS = ("/joint_command", "/piper_measured_joint_state")
REQUIRED_TOPIC_TYPE = "sensor_msgs/msg/JointState"


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_name")
    parser.add_argument("task")
    parser.add_argument("num_episodes", type=int)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode-time-s", type=float, default=60.0)
    parser.add_argument("--max-age-s", type=float, default=0.25)
    parser.add_argument("--topic-timeout-s", type=float, default=5.0)
    parser.add_argument("--resume", action="store_true")
    return parser


def parse_args(argv=None):
    args = build_parser().parse_args(argv)
    user = os.environ.get("HF_USER") or getpass.getuser()
    args.repo_id = f"{user}/{args.dataset_name}"
    args.root = args.root or DEFAULT_ROOT_BASE / args.dataset_name
    return args


def joint_state_to_vector(position):
    """Convert at least six joint positions to a seven-element float vector."""
    if len(position) < 6:
        raise ValueError("joint state must contain at least six positions")
    vector = [float(value) for value in position[:7]]
    if len(vector) == 6:
        vector.append(0.0)
    return vector


def sample_is_fresh(observation_time, action_time, now, max_age_s):
    """Return whether both timestamped messages are within the allowed age."""
    if observation_time is None or action_time is None:
        return False
    return (
        0 <= now - observation_time <= max_age_s
        and 0 <= now - action_time <= max_age_s
    )


def build_frame(observation, action, task):
    """Build one normalized dataset frame."""
    return {
        "observation.state": [float(value) for value in observation],
        "action": [float(value) for value in action],
        "task": task,
    }


class RecorderState:
    """Keep the latest valid action and observation with receipt timestamps."""

    def __init__(self, max_age_s=0.25):
        self.max_age_s = float(max_age_s)
        self.action = None
        self.action_time = None
        self.observation = None
        self.observation_time = None

    def update_action(self, position, timestamp):
        self.action = joint_state_to_vector(position)
        self.action_time = float(timestamp)

    def update_observation(self, position, timestamp):
        self.observation = joint_state_to_vector(position)
        self.observation_time = float(timestamp)

    def latest_sample(self, now):
        if not sample_is_fresh(
            self.observation_time, self.action_time, now, self.max_age_s
        ):
            return None
        return list(self.observation), list(self.action)


class VrJointRecorder(Node):
    """Subscribe to command and measured joint state topics without publishing."""

    def __init__(self, max_age_s=0.25, clock=None):
        super().__init__("vr_joint_recorder")
        self.state = RecorderState(max_age_s=max_age_s)
        self._clock = clock or time.perf_counter
        self.create_subscription(JointState, "joint_command", self.command_callback, 10)
        self.create_subscription(
            JointState,
            "piper_measured_joint_state",
            self.measured_callback,
            10,
        )

    def _warn(self, message):
        logger = self.get_logger()
        if logger is not None:
            logger.warning(message)

    def command_callback(self, message):
        try:
            self.state.update_action(message.position, self._clock())
        except ValueError as exc:
            self._warn("Ignoring invalid joint command: %s" % exc)

    def measured_callback(self, message):
        try:
            self.state.update_observation(message.position, self._clock())
        except ValueError as exc:
            self._warn("Ignoring invalid measured joint state: %s" % exc)


def wait_for_required_topics(node, timeout_s, clock=None, sleep=None):
    """Wait until both recorder topics advertise the expected message type."""
    if timeout_s < 0:
        raise ValueError("topic timeout must be non-negative")
    clock = clock or time.perf_counter
    sleep = sleep or time.sleep
    deadline = clock() + float(timeout_s)
    while True:
        advertised = dict(node.get_topic_names_and_types())
        missing = [topic for topic in REQUIRED_TOPICS if topic not in advertised]
        wrong_type = [
            topic
            for topic in REQUIRED_TOPICS
            if topic in advertised and advertised[topic] != [REQUIRED_TOPIC_TYPE]
        ]
        if not missing and not wrong_type:
            return
        if clock() >= deadline:
            details = []
            if missing:
                details.append("missing topics: " + ", ".join(missing))
            if wrong_type:
                details.append(
                    "wrong type (expected "
                    + REQUIRED_TOPIC_TYPE
                    + "): "
                    + ", ".join(
                        f"{topic} [{', '.join(advertised[topic]) or 'none'}]"
                        for topic in wrong_type
                    )
                )
            raise RuntimeError("ROS topic preflight timed out; " + "; ".join(details))
        rclpy.spin_once(node, timeout_sec=0.0)
        sleep(min(0.05, max(0.0, deadline - clock())))


def _make_dataset(args):
    dataset = getattr(args, "dataset", None)
    if dataset is not None:
        return dataset
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if getattr(args, "resume", False):
        dataset = LeRobotDataset(repo_id=args.repo_id, root=args.root)
        if dataset.meta.fps != args.fps:
            raise ValueError(
                f"dataset FPS {dataset.meta.fps} does not match requested FPS {args.fps}"
            )
        if set(dataset.meta.features) != set(DATASET_FEATURES):
            raise ValueError("dataset feature keys do not match the joint-only schema")
        return dataset

    return LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        root=args.root,
        robot_type="piper_vr",
        features=DATASET_FEATURES,
        use_videos=False,
    )


def record_episodes(args):
    """Record fresh ROS samples into fixed-FPS LeRobot episodes."""
    if rclpy is None:
        raise RuntimeError("ROS2 (rclpy) is required to record episodes")

    rclpy.init()
    node = None
    dataset = None
    try:
        node = VrJointRecorder(
            max_age_s=getattr(args, "max_age_s", 0.25),
        )
        wait_for_required_topics(
            node,
            getattr(args, "topic_timeout_s", 5.0),
        )
        dataset = _make_dataset(args)
        period = 1.0 / float(args.fps)
        for _episode in range(int(args.num_episodes)):
            episode_start = time.perf_counter()
            next_sample = episode_start
            while time.perf_counter() - episode_start < float(args.episode_time_s):
                rclpy.spin_once(node, timeout_sec=0.0)
                now = time.perf_counter()
                if now >= next_sample:
                    sample = node.state.latest_sample(now)
                    if sample is not None:
                        observation, action = sample
                        dataset.add_frame(
                            build_frame(observation, action, args.task)
                        )
                    next_sample += period
                else:
                    time.sleep(min(next_sample - now, period))
            dataset.save_episode()
    except KeyboardInterrupt:
        pass
    finally:
        if dataset is not None:
            finalize = getattr(dataset, "finalize", None)
            if finalize is not None:
                finalize()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    record_episodes(parse_args(argv))


if __name__ == "__main__":
    main()
