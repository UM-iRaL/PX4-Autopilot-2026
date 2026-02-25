"""
Keyboard teleop trajectory implementation.

Allows manual keyboard control during autonomous flight for quick testing.

Key map:
    Arrow Up    → forward  (relative to drone heading)
    Arrow Down  → backward (relative to drone heading)
    Arrow Right → right    (relative to drone heading)
    Arrow Left  → left     (relative to drone heading)
    .  (period) → +Z (up)
    ,  (comma)  → -Z (down)
    i            → +pitch (nose up)
    k            → -pitch (nose down)
    j            → -yaw (CW / right)
    l            → +yaw (CCW / left)
    u            → -roll (CW / right wing down)
    o            → +roll (CCW / left wing down)
    Space        → level attitude (reset roll/pitch/yaw to 0)
    p            → toggle push mode (4 m/s² in world -Y)

Trajectory: 13 (Keyboard Teleop)
"""

import math
from typing import Dict, Any, Optional

import numpy as np
import rospy
from tf.transformations import (quaternion_from_euler, quaternion_multiply,
                                quaternion_matrix)

from ..base import StatefulTrajectory
from utils.keyboard_handler import KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT


class KeyboardTeleopTrajectory(StatefulTrajectory):
    """Manual keyboard takeover trajectory for in-flight testing.

    Position (XY, Z) and attitude (roll, pitch, yaw) are controlled
    directly via step increments — no velocity integration.  Arrow keys
    and ,/. step the hold position.  i/k/j/l/u/o step the held Euler
    angles.  Space resets attitude to level.

    Keys are registered by the main loop calling update_key(key) — the
    trajectory never blocks on input itself.

    Trajectory ID: 13
    """

    # Default step sizes per tick at 20 Hz (overridden by config if present)
    DEFAULT_POS_STEP_XY  = 0.015  # m per tick  (0.3 m/s equivalent at 20 Hz)
    DEFAULT_POS_STEP_Z   = 0.010  # m per tick  (0.2 m/s equivalent at 20 Hz)
    DEFAULT_YAW_STEP     = 1.0    # deg per tick (20 deg/s equivalent at 20 Hz)
    DEFAULT_PITCH_STEP   = 1.0    # deg per tick
    DEFAULT_ROLL_STEP    = 1.0    # deg per tick

    def __init__(self, config, publisher=None):
        """Initialize keyboard teleop trajectory.

        Args:
            config: SetpointConfig (reads KEYBOARD_TELEOP section if present)
            publisher: MAVROSFullStatePublisher for pose access
        """
        super().__init__(config, publisher)

        kb_cfg = getattr(config, 'KEYBOARD_TELEOP', {})
        # Step sizes per tick.  Config values are rates (per-second); convert
        # to per-tick by dividing by 20 Hz.  Defaults are already per-tick.
        self._pos_step_xy  = kb_cfg.get('max_velocity_xy',   self.DEFAULT_POS_STEP_XY * 20) / 20.0
        self._pos_step_z   = kb_cfg.get('max_velocity_z',    self.DEFAULT_POS_STEP_Z * 20) / 20.0
        self._yaw_step     = math.radians(kb_cfg.get('max_yaw_rate_deg',   self.DEFAULT_YAW_STEP * 20) / 20.0)
        self._pitch_step   = math.radians(kb_cfg.get('max_pitch_rate_deg', self.DEFAULT_PITCH_STEP * 20) / 20.0)
        self._roll_step    = math.radians(kb_cfg.get('max_roll_rate_deg',  self.DEFAULT_ROLL_STEP * 20) / 20.0)

        # Internal state
        self._hold_position: Optional[np.ndarray] = None
        # Orientation stored as quaternion in NED aircraft frame [x, y, z, w]
        # to avoid gimbal lock when pitch approaches ±90°.
        self._hold_quat_ned: np.ndarray = self._canonical_quat(
            quaternion_from_euler(0, 0, 0, axes='rzyx'))

        # Current key held by main loop (None = no key / stopped)
        self._current_key: Optional[str] = None

        # Push mode: applies feedforward acceleration in world -Y (ENU) to press against surfaces
        self._push_active: bool = False
        self._push_accel: float = kb_cfg.get('push_acceleration', 9.0)  # m/s²

    @staticmethod
    def _canonical_quat(q: np.ndarray) -> np.ndarray:
        """Return quaternion with positive w (canonical form).

        q and -q represent the same rotation, but if the setpoint has
        positive w and odometry has negative w (or vice-versa) the
        controller sees a large error.  Always forcing w > 0 avoids this.
        """
        if q[3] < 0:
            return -q
        return q

    # ------------------------------------------------------------------
    # Public API — called by the main keyboard loop
    # ------------------------------------------------------------------

    @property
    def push_active(self) -> bool:
        """Whether push mode is currently active."""
        return self._push_active

    def toggle_push(self) -> bool:
        """Toggle push mode on/off. Returns the new state.

        When turning push OFF, snaps the hold setpoint to the drone's
        current actual position and orientation so that it doesn't jump
        back to the pre-push setpoint (which may be far away if the
        object moved during perching).
        """
        self._push_active = not self._push_active
        if self._push_active:
            rospy.loginfo("KeyboardTeleop: PUSH ON  (%.1f m/s² in world -Y)" % self._push_accel)
        else:
            # Snap hold setpoint to actual pose to avoid position jump
            if self.publisher is not None and self.publisher.current_pose is not None:
                p = self.publisher.current_pose.pose.position
                self._hold_position = np.array([p.x, p.y, p.z])

                # Convert ENU orientation to NED for hold quaternion
                q = self.publisher.current_pose.pose.orientation
                q_enu = np.array([q.x, q.y, q.z, q.w])
                from utils.math_utils import enu_baselink_to_ned_aircraft
                q_ned = enu_baselink_to_ned_aircraft(q_enu)
                self._hold_quat_ned = self._canonical_quat(np.array(q_ned, dtype=float))

                rospy.loginfo("KeyboardTeleop: PUSH OFF — snapped to actual pose "
                              "(%.2f, %.2f, %.2f)" % (p.x, p.y, p.z))
            else:
                rospy.loginfo("KeyboardTeleop: PUSH OFF (no pose feedback — keeping last setpoint)")
        return self._push_active

    def update_key(self, key: Optional[str]) -> None:
        """Register the currently pressed key.

        Called every iteration of the main keyboard loop.  Pass None when
        no teleop key is active (so the drone holds its current state).

        Args:
            key: One of KEY_UP/DOWN/LEFT/RIGHT, '.', ',', 'i', 'k', 'j', 'l',
                 'u', 'o', ' ' (space), or None
        """
        self._current_key = key

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_start(self) -> bool:
        """Capture current position and attitude as hold reference."""
        if self.publisher is None:
            rospy.logerr("KeyboardTeleop: no publisher — cannot start")
            return False

        if self.publisher.current_pose is not None:
            p = self.publisher.current_pose.pose.position
            self._hold_position = np.array([p.x, p.y, p.z])
            rospy.loginfo(f"KeyboardTeleop: hold from actual pose "
                          f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        else:
            pt = self.publisher.position_target
            self._hold_position = np.array([
                pt.position.x, pt.position.y, pt.position.z
            ])
            rospy.logwarn("KeyboardTeleop: no pose feedback — using commanded position")

        # Initialize hold quaternion from publisher (NED aircraft frame)
        q_ned = getattr(self.publisher, 'last_commanded_quat_ned', None)
        if q_ned is not None:
            self._hold_quat_ned = self._canonical_quat(np.array(q_ned, dtype=float))
        else:
            roll  = getattr(self.publisher, 'last_commanded_roll',  0.0)
            pitch = getattr(self.publisher, 'last_commanded_pitch', 0.0)
            yaw   = getattr(self.publisher, 'last_commanded_yaw',   0.0)
            self._hold_quat_ned = self._canonical_quat(
                np.array(quaternion_from_euler(yaw, pitch, roll, axes='rzyx')))
        self._current_key = None

        self._initialized = True
        self._active = True
        rospy.loginfo("KeyboardTeleop: started")
        rospy.loginfo("  Arrows/,/.: XYZ position  |  i/k: pitch  |  j/l: yaw  |  u/o: roll  |  Space: level")
        rospy.loginfo(f"  step XY={self._pos_step_xy*1000:.1f} mm/tick  Z={self._pos_step_z*1000:.1f} mm/tick  "
                      f"pitch/roll/yaw={math.degrees(self._pitch_step):.1f} deg/tick")
        return True

    def on_stop(self) -> None:
        self._hold_position = None
        self._hold_quat_ned = self._canonical_quat(
            np.array(quaternion_from_euler(0, 0, 0, axes='rzyx')))
        self._current_key = None
        self._push_active = False
        self._initialized = False
        self._active = False
        rospy.loginfo("KeyboardTeleop: stopped")

    # ------------------------------------------------------------------
    # Setpoint computation
    # ------------------------------------------------------------------

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute keyboard teleop setpoint at time t.

        Position keys directly step the hold position.
        Attitude keys directly step the held Euler angles.
        Space instantly levels the drone.

        Args:
            t: Elapsed time (unused — steps are applied each 20 Hz tick)

        Returns:
            Setpoint dict with position and orientation (no velocity/rates).
        """
        if self._hold_position is None:
            if not self.on_start():
                return self._safe_hover_setpoint()

        key = self._current_key

        # Space → instantly level attitude (identity quaternion)
        if key == ' ':
            self._hold_quat_ned = self._canonical_quat(
                np.array(quaternion_from_euler(0, 0, 0, axes='rzyx')))
        elif key is not None:
            self._apply_key_step(key)

        q = self._hold_quat_ned

        # Compute acceleration: if push mode is active, apply feedforward
        # acceleration in world -Y (ENU frame) to press against surface.
        if self._push_active:
            accel = {
                'ax': 0.0,
                'ay': float(-self._push_accel),
                'az': 0.0
            }
        else:
            accel = self.zero_acceleration()

        return self.build_setpoint(
            position={
                'x': float(self._hold_position[0]),
                'y': float(self._hold_position[1]),
                'z': float(self._hold_position[2])
            },
            velocity=self.zero_velocity(),
            acceleration=accel,
            orientation_quat={
                'x': float(q[0]),
                'y': float(q[1]),
                'z': float(q[2]),
                'w': float(q[3])
            },
            rates=self.zero_rates()
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _apply_key_step(self, key: str) -> None:
        """Apply a single position or orientation step for the given key.

        Position steps in XY are body-relative (forward/right) and rotated
        into ENU using the current held yaw.

        Attitude steps are applied as body-frame quaternion rotations
        (right-multiply) to avoid gimbal lock at pitch ≈ ±90°.
        """
        # Position — body-relative (forward/right/down in NED body frame)
        d_fwd = 0.0    # body-x (forward)
        d_right = 0.0  # body-y (right)
        d_down = 0.0   # body-z (down)

        if key == KEY_UP:
            d_fwd = self._pos_step_xy
        elif key == KEY_DOWN:
            d_fwd = -self._pos_step_xy
        elif key == KEY_RIGHT:
            d_right = self._pos_step_xy
        elif key == KEY_LEFT:
            d_right = -self._pos_step_xy
        # Z — along body-z axis (NED down).  '.' = up = negative body-down
        elif key == '.':
            d_down = -self._pos_step_z
        elif key == ',':
            d_down = self._pos_step_z
        # Attitude — body-frame rotations via quaternion multiply.
        # quaternion_from_euler(yaw, pitch, roll, 'rzyx') with only one
        # axis non-zero gives a rotation about that NED body axis.
        # Right-multiply = rotation in the current body frame.
        elif key == 'i':
            dq = quaternion_from_euler(0, self._pitch_step, 0, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        elif key == 'k':
            dq = quaternion_from_euler(0, -self._pitch_step, 0, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        elif key == 'j':
            dq = quaternion_from_euler(-self._yaw_step, 0, 0, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        elif key == 'l':
            dq = quaternion_from_euler(self._yaw_step, 0, 0, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        elif key == 'u':
            dq = quaternion_from_euler(0, 0, -self._roll_step, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        elif key == 'o':
            dq = quaternion_from_euler(0, 0, self._roll_step, axes='rzyx')
            self._hold_quat_ned = self._canonical_quat(
                quaternion_multiply(self._hold_quat_ned, dq))
            return
        else:
            return  # unknown key — hold

        # Rotate body-relative (forward, right, down) into ENU using the full
        # quaternion rotation matrix.  This avoids gimbal lock — at 90° pitch,
        # body-forward points down and body-down points backward, etc.
        if d_fwd != 0.0 or d_right != 0.0 or d_down != 0.0:
            # Body→NED world rotation matrix from the hold quaternion.
            R_body_to_ned = quaternion_matrix(self._hold_quat_ned)[:3, :3]
            # NED body axes: forward=[1,0,0], right=[0,1,0], down=[0,0,1]
            ned_delta = R_body_to_ned @ np.array([d_fwd, d_right, d_down])
            # NED→ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
            self._hold_position[0] += ned_delta[1]   # ENU x = NED y
            self._hold_position[1] += ned_delta[0]   # ENU y = NED x
            self._hold_position[2] += -ned_delta[2]  # ENU z = -NED z

    def _safe_hover_setpoint(self) -> Dict[str, Any]:
        q = quaternion_from_euler(0, 0, 0, axes='rzyx')  # identity
        return self.build_setpoint(
            position={'x': 0.0, 'y': 0.0, 'z': 0.7},
            velocity=self.zero_velocity(),
            acceleration=self.zero_acceleration(),
            orientation_quat={
                'x': float(q[0]), 'y': float(q[1]),
                'z': float(q[2]), 'w': float(q[3])
            },
            rates=self.zero_rates()
        )
