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



def limit_target_translation_with_wall(previous, target, minimum, maximum):
    """Apply a one-sided workspace wall: discard outward motion, allow reverse."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    minimum = np.asarray(minimum, dtype=float)
    maximum = np.asarray(maximum, dtype=float)
    if minimum.shape != (3,) or maximum.shape != (3,):
        raise ValueError("workspace bounds must be ordered 3-vectors")
    if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
        raise ValueError("workspace bounds must be finite")
    if np.any(minimum > maximum):
        raise ValueError("workspace bounds must be ordered 3-vectors")
    desired = target[:3, 3].copy()
    limited = np.clip(desired, minimum, maximum)
    previous_xyz = previous[:3, 3]
    for axis in range(3):
        if previous_xyz[axis] <= minimum[axis] and desired[axis] < minimum[axis]:
            limited[axis] = minimum[axis]
        elif previous_xyz[axis] >= maximum[axis] and desired[axis] > maximum[axis]:
            limited[axis] = maximum[axis]
    result = target.copy()
    result[:3, 3] = limited
    return result, bool(not np.allclose(desired, limited))


def limit_target_translation_with_wall(previous, target, minimum, maximum):
    """Apply a one-sided workspace wall: discard outward motion, allow reverse."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    minimum = np.asarray(minimum, dtype=float)
    maximum = np.asarray(maximum, dtype=float)
    if minimum.shape != (3,) or maximum.shape != (3,):
        raise ValueError("workspace bounds must be ordered 3-vectors")
    if not np.all(np.isfinite(minimum)) or not np.all(np.isfinite(maximum)):
        raise ValueError("workspace bounds must be finite")
    if np.any(minimum > maximum):
        raise ValueError("workspace bounds must be ordered 3-vectors")
    desired = target[:3, 3].copy()
    limited = np.clip(desired, minimum, maximum)
    previous_xyz = previous[:3, 3]
    for axis in range(3):
        if previous_xyz[axis] <= minimum[axis] and desired[axis] < minimum[axis]:
            limited[axis] = minimum[axis]
        elif previous_xyz[axis] >= maximum[axis] and desired[axis] > maximum[axis]:
            limited[axis] = maximum[axis]
    result = target.copy()
    result[:3, 3] = limited
    return result, bool(not np.allclose(desired, limited))


def limit_pose_error(reference, target, max_translation, max_rotation):
    """Limit target error relative to the current EE pose."""
    reference = _validate_pose(reference)
    target = _validate_pose(target)
    max_translation = float(max_translation)
    max_rotation = float(max_rotation)
    if max_translation <= 0 or max_rotation <= 0:
        raise ValueError("reach limits must be positive")
    result = target.copy()
    clipped = False
    delta = target[:3, 3] - reference[:3, 3]
    distance = float(np.linalg.norm(delta))
    if distance > max_translation:
        result[:3, 3] = reference[:3, 3] + delta * (max_translation / distance)
        clipped = True
    rotation_delta = reference[:3, :3].T @ target[:3, :3]
    rotation_vector = np.asarray(pin.log3(rotation_delta), dtype=float).reshape(3)
    angle = float(np.linalg.norm(rotation_vector))
    if angle > max_rotation:
        result[:3, :3] = reference[:3, :3] @ pin.exp3(
            rotation_vector * (max_rotation / angle)
        )
        clipped = True
    return result, clipped


def average_pose_window(poses):
    """Robustly average a short pose window for VR tracking noise."""
    if not poses:
        raise ValueError("pose window must not be empty")
    samples = [_validate_pose(pose) for pose in poses]
    result = samples[-1].copy()
    result[:3, 3] = np.median(
        np.stack([sample[:3, 3] for sample in samples], axis=0), axis=0
    )
    reference_rotation = samples[0][:3, :3]
    rotation_vectors = [
        np.asarray(pin.log3(reference_rotation.T @ sample[:3, :3]), dtype=float)
        for sample in samples
    ]
    result[:3, :3] = reference_rotation @ pin.exp3(
        np.mean(np.stack(rotation_vectors, axis=0), axis=0)
    )
    return result

def pose_window_is_stable(poses, max_translation_delta=0.01,
                         max_rotation_delta=0.08):
    """Check whether recent controller poses are stable enough to anchor B."""
    if max_translation_delta <= 0 or max_rotation_delta <= 0:
        raise ValueError("anchor stability limits must be positive")
    if len(poses) < 2:
        return False
    samples = [_validate_pose(pose) for pose in poses]
    reference = samples[0]
    for sample in samples[1:]:
        translation_delta = float(np.linalg.norm(
            sample[:3, 3] - reference[:3, 3]
        ))
        rotation_delta = reference[:3, :3].T @ sample[:3, :3]
        rotation_distance = float(np.linalg.norm(np.asarray(pin.log3(rotation_delta))))
        if (translation_delta > max_translation_delta or
                rotation_distance > max_rotation_delta):
            return False
    return True

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
                       fast_rotation_threshold=0.08, rotation_response_alpha=None, fast_translation_response_alpha=None, fast_rotation_response_alpha=None):
    """Filter small motion while bounding large motion for low-latency control."""
    previous = _validate_pose(previous)
    target = _validate_pose(target)
    if not 0 < response_alpha <= 1:
        raise ValueError("response_alpha must be in (0, 1]")
    rotation_alpha = response_alpha if rotation_response_alpha is None else float(rotation_response_alpha)
    if not 0 < rotation_alpha <= 1:
        raise ValueError("rotation_response_alpha must be in (0, 1]")
    fast_translation_alpha = response_alpha if fast_translation_response_alpha is None else float(fast_translation_response_alpha)
    fast_rotation_alpha = rotation_alpha if fast_rotation_response_alpha is None else float(fast_rotation_response_alpha)
    if not 0 < fast_translation_alpha <= 1:
        raise ValueError("fast_translation_response_alpha must be in (0, 1]")
    if not 0 < fast_rotation_alpha <= 1:
        raise ValueError("fast_rotation_response_alpha must be in (0, 1]")
    if max_translation_step <= 0 or max_rotation_step <= 0:
        raise ValueError("adaptive step limits must be positive")
    if fast_translation_threshold <= 0 or fast_rotation_threshold <= 0:
        raise ValueError("adaptive thresholds must be positive")

    translation_delta = target[:3, 3] - previous[:3, 3]
    translation_distance = float(np.linalg.norm(translation_delta))
    rotation_delta = previous[:3, :3].T @ target[:3, :3]
    rotation_vector = np.asarray(pin.log3(rotation_delta), dtype=float).reshape(3)
    rotation_distance = float(np.linalg.norm(rotation_vector))

    translation_fraction = float(fast_translation_alpha if translation_distance > fast_translation_threshold else response_alpha)
    rotation_fraction = float(fast_rotation_alpha if rotation_distance > fast_rotation_threshold else rotation_alpha)
    if translation_distance > fast_translation_threshold:
        translation_fraction = min(translation_fraction, float(max_translation_step) / translation_distance)
    if rotation_distance > fast_rotation_threshold:
        rotation_fraction = min(rotation_fraction, float(max_rotation_step) / rotation_distance)
    translation_fraction = min(1.0, translation_fraction)
    rotation_fraction = min(1.0, rotation_fraction)

    result = previous.copy()
    result[:3, 3] = previous[:3, 3] + translation_fraction * translation_delta
    result[:3, :3] = previous[:3, :3] @ pin.exp3(rotation_fraction * rotation_vector)
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



def apply_wrist_pivot(pose, wrist_pivot_offset_m=(0.0, 0.0, 0.0)):
    """Move the virtual control point from the controller to the wrist pivot.

    The offset is expressed in the controller's local frame, from the tracked
    controller origin to the virtual wrist pivot. Zero disables compensation.
    """
    pose = _validate_pose(pose)
    offset = np.asarray(wrist_pivot_offset_m, dtype=float)
    if offset.shape != (3,) or not np.all(np.isfinite(offset)):
        raise ValueError("wrist_pivot_offset_m must be a finite 3-vector")
    result = pose.copy()
    result[:3, 3] += pose[:3, :3] @ offset
    return result

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
    result = target.copy()
    if translation_distance <= translation_deadband:
        result[:3, 3] = previous[:3, 3]
    if rotation_distance <= rotation_deadband:
        result[:3, :3] = previous[:3, :3]
    return result


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
