"""
Landing trajectory implementation.

Safe landing with attitude stabilization phase followed by descent.

Trajectory: 7 (Landing with Controlled Descent)
"""

import math
from typing import Dict, Any, Optional

import numpy as np
import rospy

from ..base import StatefulTrajectory
from ..wait_trajectory import WaitTrajectory

# Import math utilities - handle both direct and package imports
try:
    from utils.math_utils import shortest_angular_distance, wrap_to_closest_equivalent
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from utils.math_utils import shortest_angular_distance, wrap_to_closest_equivalent


class LandingTrajectory(StatefulTrajectory):
    """Safe landing with attitude stabilization and controlled descent.

    A two-phase landing trajectory using WaitTrajectory with quintic
    polynomial interpolation:

    Phase 1 (Stabilization): Level the drone (roll/pitch to zero) while
    maintaining current position. Duration is calculated based on how
    tilted the drone is, with a maximum rotation speed limit.

    Phase 2 (Descent): Descend to ground altitude while maintaining
    level attitude and current yaw.

    The trajectory is created dynamically on first call based on the
    drone's current state (position and attitude from pose feedback).

    Trajectory handled: 7
    """

    def __init__(self, config, publisher=None):
        """Initialize landing trajectory.

        Args:
            config: SetpointConfig with LANDING parameters
            publisher: MAVROSFullStatePublisher for pose feedback
        """
        super().__init__(config, publisher)
        self._landing_traj: Optional[WaitTrajectory] = None
        self._ground_altitude = config.LANDING['ground_altitude']

        # Current pose storage (set by manager before trajectory runs)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

    @property
    def ground_altitude(self) -> float:
        """Ground altitude from config."""
        return self._ground_altitude

    def on_start(self) -> bool:
        """Initialize landing trajectory on first call.

        Creates a WaitTrajectory with waypoints based on current drone state.
        Calculates stabilization duration from current attitude.

        Returns:
            True if trajectory initialized successfully, False otherwise
        """
        if self._initialized:
            return True

        rospy.loginfo("Initializing safe landing trajectory...")

        # Get current position from publisher
        if self.publisher is None:
            rospy.logerr("Landing: No publisher available for pose feedback")
            return False

        # Use actual drone position if available, otherwise fall back to commanded position
        if self.publisher.current_pose is not None:
            start_x = self.publisher.current_pose.pose.position.x
            start_y = self.publisher.current_pose.pose.position.y
            start_z = self.publisher.current_pose.pose.position.z
            rospy.loginfo(f"Landing: Using actual position ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")
        else:
            start_x = self.publisher.position_target.position.x
            start_y = self.publisher.position_target.position.y
            start_z = self.publisher.position_target.position.z
            rospy.logwarn(f"Landing: No pose feedback, using commanded position ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")

        # Get current attitude
        roll = self.current_roll
        pitch = self.current_pitch
        start_yaw = self.current_yaw

        # Calculate stabilization duration based on attitude deviation
        roll_distance = abs(shortest_angular_distance(roll, 0.0))
        pitch_distance = abs(shortest_angular_distance(pitch, 0.0))
        max_angle_change = max(roll_distance, pitch_distance)

        # Calculate duration with rotation speed limit
        # For quintic polynomials, peak velocity is ~1.875x average
        max_rotation_speed = 0.314  # rad/s
        quintic_peak_factor = 2.0   # Safety factor
        stabilization_duration = (max_angle_change / max_rotation_speed) * quintic_peak_factor
        stabilization_duration = max(2.0, min(stabilization_duration, 15.0))

        rospy.loginfo(f"Landing: Roll={math.degrees(roll):.1f}°, Pitch={math.degrees(pitch):.1f}°, "
                     f"Stabilization time={stabilization_duration:.2f}s")

        # Wrap target angles to shortest rotational path
        target_roll = wrap_to_closest_equivalent(0.0, roll)
        target_pitch = wrap_to_closest_equivalent(0.0, pitch)
        target_yaw = wrap_to_closest_equivalent(start_yaw, start_yaw)

        # Create waypoints
        point_start = np.array([start_x, start_y, start_z])
        point_stabilized = np.array([start_x, start_y, start_z])  # Same position, leveled
        point_ground = np.array([start_x, start_y, self._ground_altitude])

        angles_start = np.array([roll, pitch, start_yaw])
        angles_stabilized = np.array([target_roll, target_pitch, target_yaw])
        angles_ground = np.array([target_roll, target_pitch, target_yaw])

        zero_vel = np.array([0.0, 0.0, 0.0])
        zero_acc = np.array([0.0, 0.0, 0.0])
        zero_omega = np.array([0.0, 0.0, 0.0])
        zero_alpha = np.array([0.0, 0.0, 0.0])

        descent_duration = self.config.LANDING['duration']

        waypoints = [
            (point_start, zero_vel, zero_acc, angles_start, zero_omega, zero_alpha, 0.0),
            (point_stabilized, zero_vel, zero_acc, angles_stabilized, zero_omega, zero_alpha, stabilization_duration),
            (point_ground, zero_vel, zero_acc, angles_ground, zero_omega, zero_alpha, stabilization_duration + descent_duration),
        ]

        # Create WaitTrajectory with quintic polynomial
        self._landing_traj = WaitTrajectory(
            init_type='quintic_polynomial',
            waypoints=waypoints,
            time_scaling_gain=0.0  # No dynamic time scaling for landing
        )

        self._initialized = True
        self._active = True
        rospy.loginfo("Landing trajectory initialized successfully")
        return True

    def on_stop(self) -> None:
        """Clean up landing trajectory for fresh restart."""
        self._landing_traj = None
        self._initialized = False
        self._active = False
        rospy.loginfo("Landing trajectory cleared")

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute landing setpoint at time t.

        Initializes trajectory on first call if needed, then delegates
        to WaitTrajectory for smooth path following.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, acceleration, orientation,
            rates, and landing_complete flag
        """
        # Initialize on first call
        if not self._initialized:
            self.on_start()

        # Return safe hover if initialization failed
        if self._landing_traj is None:
            rospy.logerr_throttle(2.0, "Landing trajectory not initialized")
            return self.build_setpoint(
                position=self.hover_position(0.7),
                velocity=self.zero_velocity(),
                orientation=self.level_orientation(),
                rates=self.zero_rates(),
                landing_complete=False
            )

        # Get state from WaitTrajectory
        try:
            x, v, a, theta, omega, alpha = self._landing_traj.get(t, robot_pos=None)
        except Exception as e:
            rospy.logerr(f"Landing trajectory get() failed: {e}")
            return self.build_setpoint(
                position={'x': 0.0, 'y': 0.0, 'z': 0.0},
                velocity=self.zero_velocity(),
                acceleration=self.zero_acceleration(),
                orientation=self.level_orientation(),
                rates=self.zero_rates(),
                landing_complete=False
            )

        # Check if landing is complete using ACTUAL drone altitude, not commanded
        # This prevents disarming while the drone is still in the air
        landing_complete = False
        if self.publisher is not None and self.publisher.current_pose is not None:
            actual_altitude = self.publisher.current_pose.pose.position.z
            # Use a small tolerance (10cm) above ground_altitude to account for sensor noise
            landing_threshold = self._ground_altitude + 0.1
            landing_complete = actual_altitude <= landing_threshold
            if landing_complete:
                rospy.loginfo(f"Landing: Actual altitude {actual_altitude:.2f}m <= threshold {landing_threshold:.2f}m")
        else:
            # Fallback to commanded position if no pose feedback available
            landing_complete = x[2] <= self._ground_altitude
            if landing_complete:
                rospy.logwarn("Landing: Using commanded position for completion check (no pose feedback)")

        return self.build_setpoint(
            position={'x': x[0], 'y': x[1], 'z': x[2]},
            velocity={'vx': v[0], 'vy': v[1], 'vz': v[2]},
            acceleration={'ax': a[0], 'ay': a[1], 'az': a[2]},
            orientation={'roll': theta[0], 'pitch': theta[1], 'yaw': theta[2]},
            rates={'rollspeed': omega[0], 'pitchspeed': omega[1], 'yawspeed': omega[2]},
            landing_complete=landing_complete
        )
