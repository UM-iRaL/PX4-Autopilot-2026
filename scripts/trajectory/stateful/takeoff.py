"""
Takeoff trajectory implementation.

Smooth takeoff with S-curve ramp and L1 controller activation.

Trajectory: 6 (Takeoff with Acceleration Ramp)
"""

import math
from typing import Dict, Any

import rospy

from ..base import StatelessTrajectory


class TakeoffTrajectory(StatelessTrajectory):
    """Smooth takeoff with S-curve position and acceleration ramp.

    Uses a cosine-based S-curve for smooth position ramping from ground
    to target altitude, with proper velocity and acceleration feedforward.

    On start, enables the L1 adaptive controller in moment-only mode
    to improve attitude control during the takeoff phase.

    Trajectory handled: 6

    The S-curve profile:
    - Position: smooth ramp from 0 to target_altitude
    - Velocity: derived from position (sine curve)
    - Acceleration: derived from velocity (cosine curve) + feedforward ramp

    Safety: Prevents re-triggering while airborne to avoid crashes.
    """

    # Minimum altitude (meters) to consider drone "airborne"
    AIRBORNE_ALTITUDE_THRESHOLD = 0.3

    def __init__(self, config, publisher=None):
        """Initialize takeoff trajectory.

        Args:
            config: SetpointConfig with TAKEOFF and TRAJECTORY_ANGLES parameters
            publisher: MAVROSFullStatePublisher for L1 controller control
        """
        super().__init__(config, publisher)
        self.takeoff_y_offset = -0.20  # 20cm back from origin
        self._aborted = False  # Safety flag: True if takeoff blocked due to already airborne

    @property
    def duration(self) -> float:
        """Takeoff duration from config."""
        return self.config.TAKEOFF['duration']

    @property
    def target_altitude(self) -> float:
        """Target altitude from config."""
        return self.config.TRAJECTORY_ANGLES['z']

    def _is_airborne(self) -> bool:
        """Check if drone is currently airborne based on altitude.

        Returns:
            True if current altitude exceeds AIRBORNE_ALTITUDE_THRESHOLD
        """
        if self.publisher is None or self.publisher.current_pose is None:
            return False
        current_z = self.publisher.current_pose.pose.position.z
        return current_z > self.AIRBORNE_ALTITUDE_THRESHOLD

    def on_start(self) -> bool:
        """Enable L1 adaptive controller when takeoff starts.

        Sets L1 to moment-only mode (force disabled) during takeoff
        to improve attitude control during the initial climb.

        Safety: Aborts if drone is already airborne to prevent crash from
        re-triggering takeoff mid-flight.

        Returns:
            True if takeoff started, False if aborted (already airborne)
        """
        # Reset abort flag at start
        self._aborted = False

        # Safety check: prevent takeoff if already airborne
        if self._is_airborne():
            current_z = self.publisher.current_pose.pose.position.z
            rospy.logwarn(
                f"TAKEOFF BLOCKED: Drone already airborne at {current_z:.2f}m. "
                f"Threshold: {self.AIRBORNE_ALTITUDE_THRESHOLD}m. "
                "Holding current position to prevent crash."
            )
            self._aborted = True
            return False

        if self.publisher is not None:
            rospy.loginfo("Takeoff: Enabling L1 adaptive controller (moment-only mode)")
            # Apply in order: channels first, then master switch
            self.publisher.set_l1_moment_enable_param(1)  # Enable moment channel
            self.publisher.set_l1_force_enable_param(0)   # Disable force channel
            self.publisher.set_l1_active_param(1)         # Enable master switch last

        return True

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute takeoff setpoint at time t.

        Uses cosine-based S-curve for smooth ramping:
        - smooth_ramp = (1 - cos(π * progress)) / 2
        - This gives 0 at t=0, 1 at t=duration, with smooth transition

        If takeoff was aborted (drone already airborne), returns a hold
        setpoint at the target altitude instead of the ramp.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, acceleration, orientation, rates
        """
        # If aborted, hold last commanded position (freeze in place)
        if self._aborted:
            hold_setpoint = self.hold_current_setpoint()
            if hold_setpoint is not None:
                return hold_setpoint
            # Fallback if publisher unavailable: hold at target altitude
            return self.build_setpoint(
                position={'x': 0.0, 'y': self.takeoff_y_offset, 'z': self.target_altitude},
                velocity=self.zero_velocity(),
                acceleration=self.zero_acceleration(),
                orientation=self.level_orientation(),
                rates=self.zero_rates()
            )

        target_alt = self.target_altitude
        accel_start = self.config.ACCELERATION.get('az', 0.0)
        accel_end = 0.0  # Always end at zero acceleration

        if t < self.duration:
            progress = t / self.duration
            smooth_ramp = (1.0 - math.cos(progress * math.pi)) / 2.0

            # Position: ramp from 0 to target altitude
            z = target_alt * smooth_ramp

            # Velocity: first derivative of position
            # d(smooth_ramp)/dt = sin(π*t/T) * π / (2*T)
            v_z = target_alt * math.sin(progress * math.pi) * math.pi / (2.0 * self.duration)

            # Acceleration: second derivative + feedforward ramp
            # d²(smooth_ramp)/dt² = cos(π*t/T) * π² / (2*T²)
            a_z_kinematic = target_alt * math.cos(progress * math.pi) * (math.pi ** 2) / (2.0 * self.duration ** 2)
            a_z_feedforward = accel_start + (accel_end - accel_start) * smooth_ramp
            a_z = a_z_kinematic + a_z_feedforward
        else:
            # Hold at target altitude after ramp completes
            z = target_alt
            v_z = 0.0
            a_z = accel_end

        return self.build_setpoint(
            position={'x': 0.0, 'y': self.takeoff_y_offset, 'z': z},
            velocity={'vx': 0.0, 'vy': 0.0, 'vz': v_z},
            acceleration={'ax': 0.0, 'ay': 0.0, 'az': a_z},
            orientation=self.level_orientation(),
            rates=self.zero_rates()
        )
