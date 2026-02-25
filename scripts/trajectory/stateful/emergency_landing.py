"""
Emergency Landing Trajectory - Reactive stabilization for out-of-control recovery.

This trajectory uses reactive control to stop spinning, then hands off to the
existing LandingTrajectory for smooth attitude correction and descent.

The approach:
1. Rate Damping: Stop any spinning by commanding zero angular rates
2. Landing: Hand off to LandingTrajectory for quintic polynomial attitude
   correction and controlled descent

Key difference from normal landing: includes rate damping phase first to
handle spinning/unstable states before attempting trajectory-based landing.

Trajectory: 12 (Emergency Landing)
"""

import math
from typing import Dict, Any, Optional

import numpy as np
import rospy

from ..base import StatefulTrajectory
from .landing import LandingTrajectory


class EmergencyLandingTrajectory(StatefulTrajectory):
    """Emergency landing with rate damping followed by standard landing sequence.

    Unlike jumping straight to LandingTrajectory, this first stops any spinning
    by commanding zero angular rates while holding current attitude. Once rates
    are damped, it creates and delegates to a LandingTrajectory for smooth
    quintic polynomial attitude correction and descent.

    Recovery Phases:
        Phase 1 (Rate Damping): Command zero angular rates while holding current
            attitude. This stops any spinning before attempting corrections.

        Phase 2 (Landing): Delegate to LandingTrajectory which uses quintic
            polynomial interpolation for smooth attitude correction and descent.

    The trajectory continuously monitors actual state during rate damping and
    only transitions to landing once the drone is stable enough.
    """

    # Phase identifiers
    PHASE_RATE_DAMPING = 1
    PHASE_LANDING = 2

    def __init__(self, config, publisher=None, manager=None):
        """Initialize emergency landing trajectory.

        Args:
            config: SetpointConfig with EMERGENCY_LANDING parameters
            publisher: MAVROSFullStatePublisher for pose feedback
            manager: TrajectoryManager for accessing current state
        """
        super().__init__(config, publisher)
        self.manager = manager

        # Load configuration with defaults
        emergency_config = getattr(config, 'EMERGENCY_LANDING', {})

        # Rate damping parameters
        self.rate_damping_threshold = emergency_config.get('rate_damping_threshold', 0.3)  # rad/s
        self.max_rate_command = emergency_config.get('max_rate_command', 0.5)  # rad/s

        # Safety parameters
        self.max_stabilization_time = emergency_config.get('max_stabilization_time', 15.0)  # seconds

        # State tracking
        self.current_phase = self.PHASE_RATE_DAMPING
        self.phase_start_time = 0.0
        self.target_position: Optional[np.ndarray] = None

        # Landing trajectory (created after rate damping)
        self._landing_trajectory: Optional[LandingTrajectory] = None
        self._landing_start_time = 0.0

    def on_start(self) -> bool:
        """Initialize emergency landing on activation.

        Captures current position as hold target and initializes phase tracking.

        Returns:
            True (always succeeds - emergency landing must work)
        """
        rospy.logwarn("=" * 60)
        rospy.logwarn("EMERGENCY LANDING ACTIVATED")
        rospy.logwarn("Rate damping followed by standard landing sequence")
        rospy.logwarn("=" * 60)

        # Capture current position as target (will hold during rate damping)
        if self.manager is not None and self.manager.current_position is not None:
            self.target_position = self.manager.current_position.copy()
            rospy.loginfo(f"Emergency landing: Hold position ({self.target_position[0]:.2f}, "
                         f"{self.target_position[1]:.2f}, {self.target_position[2]:.2f})")
        elif self.publisher is not None and self.publisher.current_pose is not None:
            pose = self.publisher.current_pose.pose.position
            self.target_position = np.array([pose.x, pose.y, pose.z])
        else:
            # Fallback to origin at safe altitude
            self.target_position = np.array([0.0, 0.0, 0.7])
            rospy.logwarn("Emergency landing: No position feedback, using origin")

        # Initialize phase tracking
        self.current_phase = self.PHASE_RATE_DAMPING
        self.phase_start_time = rospy.Time.now().to_sec()
        self._landing_trajectory = None
        self._landing_start_time = 0.0

        self._initialized = True
        self._active = True

        rospy.loginfo("Emergency landing: Starting Phase 1 (Rate Damping)")
        return True

    def on_stop(self) -> None:
        """Clean up emergency landing state."""
        if self._landing_trajectory is not None:
            self._landing_trajectory.on_stop()
        self._landing_trajectory = None
        self._initialized = False
        self._active = False
        self.current_phase = self.PHASE_RATE_DAMPING
        self.target_position = None
        rospy.loginfo("Emergency landing: Trajectory stopped")

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute reactive setpoint based on current drone state.

        This is called every control loop iteration. During rate damping,
        setpoints are computed reactively. After rates are damped, we delegate
        to the standard LandingTrajectory.

        Args:
            t: Elapsed time since trajectory start (used for timeout)

        Returns:
            Setpoint dict with position, velocity, orientation, rates, and flags
        """
        if not self._initialized:
            self.on_start()

        # Get current state from manager
        if self.manager is None:
            rospy.logerr_throttle(1.0, "Emergency landing: No manager reference")
            return self._fallback_setpoint()

        current_rates = self.manager.current_angular_velocity
        current_pos = self.manager.current_position

        # Check for timeout - force landing if rate damping taking too long
        elapsed = t
        if elapsed > self.max_stabilization_time and self.current_phase == self.PHASE_RATE_DAMPING:
            rospy.logwarn(f"Emergency landing: Rate damping timeout ({elapsed:.1f}s), forcing landing")
            self._transition_to_landing(t)

        # Compute setpoint based on current phase
        if self.current_phase == self.PHASE_RATE_DAMPING:
            return self._rate_damping_setpoint(current_rates, current_pos, t)

        elif self.current_phase == self.PHASE_LANDING:
            return self._landing_setpoint(t)

        return self._fallback_setpoint()

    def _rate_damping_setpoint(self, rates: Optional[np.ndarray],
                                pos: Optional[np.ndarray], t: float) -> Dict:
        """Phase 1: Dampen angular rates by commanding zero rates.

        Holds current attitude while commanding zero angular velocity.
        This stops any spinning before attempting attitude corrections.
        """
        # Check if rates are available and damped
        if rates is not None:
            rate_magnitude = np.linalg.norm(rates)

            if rate_magnitude < self.rate_damping_threshold:
                rospy.loginfo(f"Emergency landing: Rates damped ({math.degrees(rate_magnitude):.1f}°/s < "
                             f"{math.degrees(self.rate_damping_threshold):.1f}°/s)")
                self._transition_to_landing(t)
            else:
                rospy.loginfo_throttle(1.0, f"Emergency landing: Damping rates "
                                       f"({math.degrees(rate_magnitude):.1f}°/s)")

        # Hold current attitude, command zero rates.
        # Use quaternion to avoid gimbal lock jitter at pitch = ±90°.
        q_ned = self.manager.current_orientation_quat
        if q_ned is not None:
            return self._build_stabilization_setpoint_quat(
                target_quat_ned=q_ned,
                target_rates=np.array([0.0, 0.0, 0.0]),
                current_pos=pos
            )
        else:
            # Fallback to Euler angles if quaternion unavailable
            return self._build_stabilization_setpoint(
                target_roll=self.manager.current_roll,
                target_pitch=self.manager.current_pitch,
                target_yaw=self.manager.current_yaw,
                target_rates=np.array([0.0, 0.0, 0.0]),
                current_pos=pos
            )

    def _transition_to_landing(self, t: float) -> None:
        """Transition from rate damping to landing phase."""
        rospy.loginfo("Emergency landing: Starting Phase 2 (Landing Sequence)")

        # Create landing trajectory
        self._landing_trajectory = LandingTrajectory(self.config, self.publisher)

        # Set current attitude so landing trajectory knows where to start from
        self._landing_trajectory.current_roll = self.manager.current_roll
        self._landing_trajectory.current_pitch = self.manager.current_pitch
        self._landing_trajectory.current_yaw = self.manager.current_yaw

        # Initialize the landing trajectory
        self._landing_trajectory.on_start()

        # Track when landing started
        self._landing_start_time = t
        self.current_phase = self.PHASE_LANDING
        self.phase_start_time = rospy.Time.now().to_sec()

    def _landing_setpoint(self, t: float) -> Dict:
        """Phase 2: Delegate to standard landing trajectory."""
        if self._landing_trajectory is None:
            rospy.logerr("Emergency landing: Landing trajectory not initialized")
            return self._fallback_setpoint()

        # Calculate time relative to landing start
        landing_t = t - self._landing_start_time

        # Get setpoint from landing trajectory
        return self._landing_trajectory.get_setpoint(landing_t)

    def _build_stabilization_setpoint(self, target_roll: float, target_pitch: float,
                                       target_yaw: float, target_rates: np.ndarray,
                                       current_pos: Optional[np.ndarray]) -> Dict:
        """Build setpoint for rate damping phase.

        During stabilization, we hold position while focusing on stopping rotation.
        """
        # Position: hold target or current
        if self.target_position is not None:
            pos_x = self.target_position[0]
            pos_y = self.target_position[1]
            pos_z = self.target_position[2]
        elif current_pos is not None:
            pos_x = current_pos[0]
            pos_y = current_pos[1]
            pos_z = current_pos[2]
        else:
            pos_x, pos_y, pos_z = 0.0, 0.0, 0.7

        return self.build_setpoint(
            position={'x': pos_x, 'y': pos_y, 'z': pos_z},
            velocity={'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            orientation={'roll': target_roll, 'pitch': target_pitch, 'yaw': target_yaw},
            rates={'rollspeed': target_rates[0], 'pitchspeed': target_rates[1],
                   'yawspeed': target_rates[2]},
            landing_complete=False,
            use_explicit_rates=True
        )

    def _build_stabilization_setpoint_quat(self, target_quat_ned: np.ndarray,
                                              target_rates: np.ndarray,
                                              current_pos: Optional[np.ndarray]) -> Dict:
        """Build setpoint for rate damping phase using quaternion orientation.

        Uses quaternion directly to avoid gimbal lock jitter at pitch = ±90°
        that occurs when round-tripping through Euler angles.
        """
        # Position: hold target or current
        if self.target_position is not None:
            pos_x = self.target_position[0]
            pos_y = self.target_position[1]
            pos_z = self.target_position[2]
        elif current_pos is not None:
            pos_x = current_pos[0]
            pos_y = current_pos[1]
            pos_z = current_pos[2]
        else:
            pos_x, pos_y, pos_z = 0.0, 0.0, 0.7

        return self.build_setpoint(
            position={'x': pos_x, 'y': pos_y, 'z': pos_z},
            velocity={'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            orientation_quat={'x': float(target_quat_ned[0]), 'y': float(target_quat_ned[1]),
                              'z': float(target_quat_ned[2]), 'w': float(target_quat_ned[3])},
            rates={'rollspeed': target_rates[0], 'pitchspeed': target_rates[1],
                   'yawspeed': target_rates[2]},
            landing_complete=False,
            use_explicit_rates=True
        )

    def _fallback_setpoint(self) -> Dict:
        """Fallback setpoint when state is unavailable."""
        rospy.logwarn_throttle(1.0, "Emergency landing: Using fallback setpoint")
        return self.build_setpoint(
            position={'x': 0.0, 'y': 0.0, 'z': 0.5},
            velocity=self.zero_velocity(),
            orientation=self.level_orientation(),
            rates=self.zero_rates(),
            landing_complete=False
        )

    def get_phase_name(self) -> str:
        """Get human-readable name of current phase."""
        phase_names = {
            self.PHASE_RATE_DAMPING: "Rate Damping",
            self.PHASE_LANDING: "Landing Sequence"
        }
        return phase_names.get(self.current_phase, "Unknown")
