from __future__ import annotations

import math
import numpy as np
import pinocchio as pin


def xyzrpy_matrix(x, y, z, roll, pitch, yaw):
    ca, sa = math.cos(yaw), math.sin(yaw)
    cb, sb = math.cos(pitch), math.sin(pitch)
    cc, sc = math.cos(roll), math.sin(roll)
    out = np.eye(4)
    out[:3, :3] = [
        [ca * cb, ca * sb * sc - sa * cc, sa * sc + ca * sb * cc],
        [sa * cb, sa * sb * sc + ca * cc, sa * sb * cc - ca * sc],
        [-sb, cb * sc, cb * cc],
    ]
    out[:3, 3] = [x, y, z]
    return out


def matrix_to_xyzrpy(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return [
        float(matrix[0, 3]), float(matrix[1, 3]), float(matrix[2, 3]),
        math.atan2(matrix[2, 1], matrix[2, 2]),
        math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0))),
        math.atan2(matrix[1, 0], matrix[0, 0]),
    ]


def quest_to_robot_transform(transform):
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    axis_adjust = np.array(
        [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
        dtype=float,
    )
    rotation_adjust = xyzrpy_matrix(0, 0, 0, -math.pi, 0, -math.pi / 2)
    return axis_adjust @ transform @ rotation_adjust


def relative_pose(base_pose, current_pose, origin_pose=None):
    zero = xyzrpy_matrix(0.19, 0.0, 0.2, 0, 0, 0) if origin_pose is None else _validate_pose(origin_pose)
    return zero @ np.linalg.inv(base_pose) @ current_pose


def limit_target_translation(target, scale=1.0,
                             minimum=(0.02, -0.50, 0.02),
                             maximum=(0.75, 0.65, 0.75),
                             origin=(0.19, 0.0, 0.2)):
    """Scale and clamp target position while preserving its orientation."""
    target = np.asarray(target, dtype=float).copy()
    if target.shape != (4, 4):
        raise ValueError("target must be a 4x4 homogeneous transform")
    if scale <= 0:
        raise ValueError("scale must be positive")
    minimum = np.asarray(minimum, dtype=float)
    maximum = np.asarray(maximum, dtype=float)
    if minimum.shape != (3,) or maximum.shape != (3,) or np.any(minimum > maximum):
        raise ValueError("workspace bounds must be ordered 3-vectors")
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        raise ValueError("origin must be a finite 3-vector")
    desired = origin + scale * (target[:3, 3] - origin)
    clipped_target = np.clip(desired, minimum, maximum)
    target[:3, 3] = clipped_target
    return target, bool(not np.allclose(desired, clipped_target))


def limit_target_translation_with_margin(target, scale=1.0,
                                         minimum=(0.02, -0.50, 0.02),
                                         maximum=(0.75, 0.65, 0.75),
                                         origin=(0.19, 0.0, 0.2),
                                         margin=0.03):
    """Clamp targets inside a safety margin to avoid boundary chatter."""
    if margin < 0:
        raise ValueError("workspace margin must be non-negative")
    minimum = np.asarray(minimum, dtype=float)
    maximum = np.asarray(maximum, dtype=float)
    if np.any(maximum - minimum <= 2.0 * margin):
        raise ValueError("workspace margin leaves no usable workspace")
    return limit_target_translation(
        target, scale=scale,
        minimum=minimum + margin,
        maximum=maximum - margin,
        origin=origin,
    )


def _validate_pose(target):
    target = np.asarray(target, dtype=float)
    if target.shape != (4, 4) or not np.all(np.isfinite(target)):
        raise ValueError("target must be a finite 4x4 homogeneous transform")
    return target.copy()


def scale_target_translation(target, scale=1.0, origin=(0.19, 0.0, 0.2)):
    target = _validate_pose(target)
    if scale <= 0:
        raise ValueError("scale must be positive")
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,):
        raise ValueError("origin must be a 3-vector")
    target[:3, 3] = origin + float(scale) * (target[:3, 3] - origin)
    return target


def scale_target_rotation(target, scale=1.0):
    target = _validate_pose(target)
    if scale <= 0:
        raise ValueError("scale must be positive")
    target[:3, :3] = pin.exp3(float(scale) * pin.log3(target[:3, :3]))
    return target


def smooth_pose(previous, target, alpha=0.25):
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    if not 0 < alpha <= 1:
        raise ValueError("alpha must be in (0, 1]")
    result = previous.copy()
    result[:3, 3] = (1.0 - alpha) * previous[:3, 3] + alpha * target[:3, 3]
    delta = previous[:3, :3].T @ target[:3, :3]
    result[:3, :3] = previous[:3, :3] @ pin.exp3(alpha * pin.log3(delta))
    return result


def adaptive_pose_step(previous, target, response_alpha=0.18,
                       max_translation_step=0.03,
                       max_rotation_step=0.16,
                       fast_translation_threshold=0.015,
                       fast_rotation_threshold=0.08):
    """Filter small motion while bounding large motion for low-latency control."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    if not 0 < response_alpha <= 1:
        raise ValueError("response_alpha must be in (0, 1]")
    if max_translation_step <= 0 or max_rotation_step <= 0:
        raise ValueError("adaptive step limits must be positive")
    if fast_translation_threshold <= 0 or fast_rotation_threshold <= 0:
        raise ValueError("adaptive thresholds must be positive")

    translation_delta = target[:3, 3] - previous[:3, 3]
    translation_distance = float(np.linalg.norm(translation_delta))
    rotation_delta = previous[:3, :3].T @ target[:3, :3]
    rotation_vector = np.asarray(pin.log3(rotation_delta), dtype=float).reshape(3)
    rotation_distance = float(np.linalg.norm(rotation_vector))

    fractions = [float(response_alpha)]
    if translation_distance > fast_translation_threshold:
        fractions.append(float(max_translation_step) / translation_distance)
    if rotation_distance > fast_rotation_threshold:
        fractions.append(float(max_rotation_step) / rotation_distance)
    fraction = min(1.0, *fractions)

    result = previous.copy()
    result[:3, 3] = previous[:3, 3] + fraction * translation_delta
    result[:3, :3] = previous[:3, :3] @ pin.exp3(fraction * rotation_vector)
    return result


def step_pose_towards(previous, target, max_translation_step=0.02,
                      max_rotation_step=0.12):
    """Move a Cartesian target by bounded translation/rotation increments."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    if max_translation_step <= 0 or max_rotation_step <= 0:
        raise ValueError("Cartesian step limits must be positive")

    translation_delta = target[:3, 3] - previous[:3, 3]
    translation_distance = float(np.linalg.norm(translation_delta))
    rotation_delta = previous[:3, :3].T @ target[:3, :3]
    rotation_vector = np.asarray(pin.log3(rotation_delta), dtype=float).reshape(3)
    rotation_distance = float(np.linalg.norm(rotation_vector))

    fractions = [1.0]
    if translation_distance > 1e-9:
        fractions.append(float(max_translation_step) / translation_distance)
    if rotation_distance > 1e-9:
        fractions.append(float(max_rotation_step) / rotation_distance)
    fraction = min(fractions)

    result = previous.copy()
    result[:3, 3] = previous[:3, 3] + fraction * translation_delta
    result[:3, :3] = previous[:3, :3] @ pin.exp3(fraction * rotation_vector)
    return result


def limit_input_pose_jump(previous_input, current_input,
                          max_translation_step=0.02,
                          max_rotation_step=0.12):
    """Limit one incoming VR pose jump before it reaches IK."""
    return step_pose_towards(
        previous_input,
        current_input,
        max_translation_step=max_translation_step,
        max_rotation_step=max_rotation_step,
    )


def pose_jump_exceeds(previous, current, translation_threshold=0.15,
                      rotation_threshold=0.6):
    """Detect tracking relocalization jumps before incremental limiting."""
    previous = _validate_pose(previous)
    current = _validate_pose(current)
    if translation_threshold <= 0 or rotation_threshold <= 0:
        raise ValueError("jump thresholds must be positive")
    translation_distance = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]))
    rotation = previous[:3, :3].T @ current[:3, :3]
    rotation_distance = float(np.linalg.norm(np.asarray(pin.log3(rotation))))
    return (translation_distance > translation_threshold or
            rotation_distance > rotation_threshold)


def apply_pose_deadband(previous, target, translation_deadband=0.002,
                        rotation_deadband=0.02):
    """Ignore small VR noise while preserving larger intentional motion."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    if translation_deadband < 0 or rotation_deadband < 0:
        raise ValueError("deadband values must be non-negative")
    translation_distance = float(np.linalg.norm(target[:3, 3] - previous[:3, 3]))
    rotation = previous[:3, :3].T @ target[:3, :3]
    rotation_distance = float(np.linalg.norm(np.asarray(pin.log3(rotation))))
    if (translation_distance <= translation_deadband and
            rotation_distance <= rotation_deadband):
        return previous.copy()
    return target.copy()


def apply_incremental_pose_delta(target, previous_input, current_input,
                                 translation_scale=1.0,
                                 rotation_scale=1.0):
    """Accumulate one bounded-time-step Quest pose delta onto a robot target."""
    target = _validate_pose(target)
    previous_input = _validate_pose(previous_input)
    current_input = _validate_pose(current_input)
    if translation_scale <= 0 or rotation_scale <= 0:
        raise ValueError("incremental scales must be positive")

    result = target.copy()
    result[:3, 3] += float(translation_scale) * (
        current_input[:3, 3] - previous_input[:3, 3]
    )
    rotation_delta = previous_input[:3, :3].T @ current_input[:3, :3]
    result[:3, :3] = result[:3, :3] @ pin.exp3(
        float(rotation_scale) * pin.log3(rotation_delta)
    )
    return result
