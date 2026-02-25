"""
Math utility functions for quaternion conversions and angle operations.

This module provides utility functions for converting between different coordinate frames
(NED aircraft and ENU base_link) and performing angle wrapping operations.
"""

import math
import numpy as np
from tf.transformations import quaternion_from_euler, quaternion_multiply


def ned_aircraft_to_enu_baselink(q_ned_aircraft):
    """
    Convert quaternion from NED aircraft frame to ENU base_link frame for MAVROS.

    MAVROS applies transformations internally in setpoint_raw.cpp:
    auto ned_desired_orientation = ftf::transform_orientation_enu_ned(
        ftf::transform_orientation_baselink_aircraft(desired_orientation));

    Which expands to:
    q_temp = desired_orientation * AIRCRAFT_BASELINK_Q
    q_ned_aircraft = NED_ENU_Q * q_temp

    Where:
    - AIRCRAFT_BASELINK_Q = [1, 0, 0, 0] (180° rotation about X)
    - NED_ENU_Q = quaternion_from_rpy(PI, 0, PI/2)

    To find the inverse transformation:
    Given: q_ned_aircraft = NED_ENU_Q * (q_enu_baselink * AIRCRAFT_BASELINK_Q)
    Solve for q_enu_baselink:

    q_enu_baselink * AIRCRAFT_BASELINK_Q = NED_ENU_Q^-1 * q_ned_aircraft
    q_enu_baselink = (NED_ENU_Q^-1 * q_ned_aircraft) * AIRCRAFT_BASELINK_Q^-1

    Args:
        q_ned_aircraft: Quaternion [x, y, z, w] in NED aircraft frame (FRD)

    Returns:
        Quaternion [x, y, z, w] in ENU base_link frame (FLU)
    """
    # Compute NED_ENU_Q = quaternion_from_rpy(PI, 0, PI/2)
    q_ned_enu = quaternion_from_euler(math.pi, 0, math.pi/2, axes='sxyz')  # [x, y, z, w]
    # Inverse is the conjugate (for unit quaternions)
    q_enu_ned = np.array([-q_ned_enu[0], -q_ned_enu[1], -q_ned_enu[2], q_ned_enu[3]])

    # AIRCRAFT_BASELINK_Q = [1, 0, 0, 0] (180° about X)
    # Its inverse is itself (180° rotation is self-inverse)
    q_baselink_aircraft = np.array([1.0, 0.0, 0.0, 0.0])  # [x, y, z, w]

    # Apply inverse: q_enu_baselink = (q_enu_ned * q_ned_aircraft) * q_baselink_aircraft
    q_temp = quaternion_multiply(q_enu_ned, q_ned_aircraft)
    q_enu_baselink = quaternion_multiply(q_temp, q_baselink_aircraft)

    return q_enu_baselink


def enu_baselink_to_ned_aircraft(q_enu_baselink):
    """
    Convert quaternion from ENU base_link frame to NED aircraft frame.
    This is the inverse of ned_aircraft_to_enu_baselink.

    Forward math was: q_enu_bl = q_ned_enu * q_ned_ac * q_ac_bl
    Inverse math is:  q_ned_ac = (q_ned_enu)^-1 * q_enu_bl * (q_ac_bl)^-1

    Args:
        q_enu_baselink: Quaternion [x, y, z, w] or list in ENU base_link frame (FLU)

    Returns:
        Quaternion [x, y, z, w] in NED aircraft frame (FRD)
    """
    # NED to ENU rotation: RPY(PI, 0, PI/2)
    q_ned_enu = quaternion_from_euler(math.pi, 0, math.pi/2, axes='sxyz')
    # Inverse is the conjugate
    q_enu_ned_inv = np.array([-q_ned_enu[0], -q_ned_enu[1], -q_ned_enu[2], q_ned_enu[3]])

    # Aircraft to BaseLink rotation: 180 deg about X
    # Its inverse is itself
    q_baselink_aircraft_inv = np.array([1.0, 0.0, 0.0, 0.0])

    # Apply Inverse: q_ned_ac = q_enu_ned_inv * q_enu_bl * q_baselink_aircraft_inv
    q_enu_bl = np.array(q_enu_baselink)
    q_temp = quaternion_multiply(q_enu_bl, q_baselink_aircraft_inv)
    q_ned_aircraft = quaternion_multiply(q_enu_ned_inv, q_temp)

    return q_ned_aircraft


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle in radians to [-π, π]."""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad <= -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def shortest_angular_distance(from_angle: float, to_angle: float) -> float:
    """
    Calculate the shortest angular distance from from_angle to to_angle.

    Args:
        from_angle: Starting angle in radians
        to_angle: Target angle in radians

    Returns:
        Shortest angular distance in radians (can be negative)
        Positive means counterclockwise rotation, negative means clockwise

    Example:
        shortest_angular_distance(170°, 0°) = -170° (rotate clockwise)
        shortest_angular_distance(0°, 170°) = 170° (rotate counterclockwise)
        shortest_angular_distance(170°, -170°) = 20° (not 340°)
    """
    diff = to_angle - from_angle
    # Wrap to [-π, π]
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


def wrap_to_closest_equivalent(target_angle: float, reference_angle: float) -> float:
    """
    Wrap target_angle to be on the same 2π branch as reference_angle.
    This ensures linear interpolation takes the shortest path.

    Args:
        target_angle: The angle to wrap (radians)
        reference_angle: The reference angle (radians)

    Returns:
        Equivalent angle to target_angle that's closest to reference_angle

    Example:
        wrap_to_closest_equivalent(-3.0, 3.0) = 3.28  (wraps -172° to 188°)
        This makes interpolation from 3.0 to 3.28 only rotate 16° instead of 344°
    """
    # Add/subtract 2π until target is within π of reference
    while target_angle - reference_angle > math.pi:
        target_angle -= 2 * math.pi
    while target_angle - reference_angle < -math.pi:
        target_angle += 2 * math.pi
    return target_angle


def euler_rates_to_body_rates_flu(roll, pitch, roll_rate, pitch_rate, yaw_rate):
    """
    Convert ZYX Euler angle rates to body-frame angular velocity in FLU.

    Uses the kinematic relation for intrinsic ZYX (yaw-pitch-roll) convention:
        p_frd =  roll_rate - yaw_rate * sin(pitch)
        q_frd =  pitch_rate * cos(roll) + yaw_rate * sin(roll) * cos(pitch)
        r_frd = -pitch_rate * sin(roll) + yaw_rate * cos(roll) * cos(pitch)

    Then converts FRD -> FLU: [p, -q, -r]

    This has a singularity at pitch = ±90° (cos(pitch) = 0). Callers should
    use body_rates_from_quaternions() as a fallback near the singularity.

    Args:
        roll: Current roll angle (radians)
        pitch: Current pitch angle (radians)
        roll_rate: d(roll)/dt (rad/s)
        pitch_rate: d(pitch)/dt (rad/s)
        yaw_rate: d(yaw)/dt (rad/s)

    Returns:
        np.array([wx, wy, wz]) body angular velocity in FLU frame (rad/s)
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)

    # Body rates in FRD
    p_frd = roll_rate - yaw_rate * sp
    q_frd = pitch_rate * cr + yaw_rate * sr * cp
    r_frd = -pitch_rate * sr + yaw_rate * cr * cp

    # FRD -> FLU
    return np.array([p_frd, -q_frd, -r_frd])


def body_rates_from_quaternions(q_prev, q_curr, dt):
    """
    Compute body-frame angular velocity from two consecutive quaternions.

    Uses the quaternion derivative relation: omega_body = 2 * Im(q_prev^{-1} * q_curr) / dt
    This avoids Euler angle singularities entirely and works at any orientation
    including pitch = ±90°.

    Args:
        q_prev: Previous quaternion [x, y, z, w] in NED aircraft frame
        q_curr: Current quaternion [x, y, z, w] in NED aircraft frame
        dt: Time step between the two quaternions (seconds)

    Returns:
        np.array([wx, wy, wz]) body angular velocity in FLU frame (rad/s)
    """
    q_prev = np.array(q_prev, dtype=float)
    q_curr = np.array(q_curr, dtype=float)

    # Normalize
    q_prev = q_prev / np.linalg.norm(q_prev)
    q_curr = q_curr / np.linalg.norm(q_curr)

    # Ensure shortest path (avoid 2pi jumps from quaternion sign flip)
    if np.dot(q_prev, q_curr) < 0:
        q_curr = -q_curr

    # q_diff = q_prev^{-1} * q_curr  (relative rotation in body frame)
    # For unit quaternions, inverse is conjugate: q^{-1} = [-x, -y, -z, w]
    q_prev_inv = np.array([-q_prev[0], -q_prev[1], -q_prev[2], q_prev[3]])
    q_diff = quaternion_multiply(q_prev_inv, q_curr)

    # For small rotations, q_diff ≈ [wx*dt/2, wy*dt/2, wz*dt/2, 1]
    # Body angular velocity in FRD: omega = 2 * Im(q_diff) / dt
    if dt > 1e-10:
        wx_frd = 2.0 * q_diff[0] / dt
        wy_frd = 2.0 * q_diff[1] / dt
        wz_frd = 2.0 * q_diff[2] / dt
    else:
        wx_frd = 0.0
        wy_frd = 0.0
        wz_frd = 0.0

    # FRD -> FLU for MAVROS
    return np.array([wx_frd, -wy_frd, -wz_frd])


def quaternion_angular_distance(q1, q2) -> float:
    """
    Compute the angular distance between two quaternions.

    Uses the formula: θ = 2 * arccos(|q1 · q2|)
    This gives the minimum angle of rotation needed to go from q1 to q2,
    avoiding gimbal lock issues that arise with Euler angle comparisons.

    Args:
        q1: First quaternion [x, y, z, w] or array-like
        q2: Second quaternion [x, y, z, w] or array-like

    Returns:
        Angular distance in radians [0, π]
    """
    q1 = np.array(q1)
    q2 = np.array(q2)

    # Normalize quaternions (in case they aren't already)
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute dot product
    dot = np.abs(np.dot(q1, q2))

    # Clamp to valid range for arccos (handle numerical errors)
    dot = np.clip(dot, 0.0, 1.0)

    # Angular distance
    return 2.0 * math.acos(dot)


def vector_magnitude(v) -> float:
    """
    Compute the magnitude (Euclidean norm) of a vector.

    Args:
        v: Vector as array-like [x, y, z] or similar

    Returns:
        Magnitude of the vector
    """
    return np.linalg.norm(np.array(v))
