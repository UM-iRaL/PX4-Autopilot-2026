"""
Angular sweep trajectory implementations.

Handles single-axis angular sweeps (roll, pitch, yaw) with optional
two-step mode for pitch. Uses parameterization to avoid code duplication.

Trajectories: 1 (roll), 2 (pitch), 3 (yaw), 8 (pitch two-step)
"""

import math
from typing import Dict, Any, Tuple

from ..base import StatelessTrajectory


class AngularSweepTrajectory(StatelessTrajectory):
    """Single-axis angular sweep with configurable axis and mode.

    A parameterized trajectory class that handles all single-axis angular
    sweeps. Instead of duplicating code for roll, pitch, yaw, and two-step
    pitch, this class accepts parameters to configure the behavior.

    Trajectories handled:
    - Trajectory 1: Roll sweep (axis='roll')
    - Trajectory 2: Pitch sweep (axis='pitch', use_quaternion=True)
    - Trajectory 3: Yaw sweep (axis='yaw')
    - Trajectory 8: Pitch two-step (axis='pitch', two_step=True, use_quaternion=True)

    Example:
        # Trajectory 1: Roll sweep
        traj = AngularSweepTrajectory(config, axis='roll')

        # Trajectory 8: Two-step pitch
        traj = AngularSweepTrajectory(config, axis='pitch', two_step=True, use_quaternion=True)
    """

    def __init__(self, config, axis: str = 'roll',
                 two_step: bool = False, use_quaternion: bool = False,
                 use_current_position: bool = False, publisher=None):
        """Initialize angular sweep trajectory.

        Args:
            config: SetpointConfig with TRAJECTORY_ANGLES parameters
            axis: 'roll', 'pitch', or 'yaw'
            two_step: If True, sweep up -> pause -> sweep down (for trajectory 8)
            use_quaternion: If True, return quaternion instead of euler angles
            use_current_position: If True, start at current position instead of origin
            publisher: Optional MAVROSFullStatePublisher
        """
        super().__init__(config, publisher)
        self.axis = axis
        self.two_step = two_step
        self.use_quaternion = use_quaternion
        self.use_current_position = use_current_position
        self._start_position = None

    def on_start(self) -> bool:
        """Capture current position if use_current_position is enabled."""
        if self.use_current_position and self.publisher is not None:
            pt = getattr(self.publisher, 'position_target', None)
            if pt is not None:
                self._start_position = {
                    'x': pt.position.x,
                    'y': pt.position.y,
                    'z': pt.position.z
                }
        return True

    @property
    def angle_min(self) -> float:
        """Minimum angle in radians from config."""
        return math.radians(self.config.TRAJECTORY_ANGLES[f'{self.axis}_min'])

    @property
    def angle_max(self) -> float:
        """Maximum angle in radians from config."""
        return math.radians(self.config.TRAJECTORY_ANGLES[f'{self.axis}_max'])

    @property
    def duration(self) -> float:
        """Trajectory duration - doubled for two-step mode."""
        base_duration = self.config.TRAJECTORY_ANGLES['duration']
        return base_duration * 2.0 if self.two_step else base_duration

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute angular sweep setpoint at time t.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, orientation/quaternion, rates
        """
        if self.two_step:
            angle, angular_speed = self._two_step_interpolation(t)
        else:
            angle, angular_speed = self._linear_interpolation(t)

        # Use captured start position or default hover position
        position = self._start_position if self._start_position is not None else self.hover_position()

        # Build orientation (euler or quaternion)
        if self.use_quaternion and self.axis == 'pitch':
            return self.build_setpoint(
                position=position,
                velocity=self.zero_velocity(),
                orientation_quat=self.make_quaternion_y_axis(angle),
                rates=self._make_rates(angular_speed)
            )
        else:
            return self.build_setpoint(
                position=position,
                velocity=self.zero_velocity(),
                orientation=self._make_orientation(angle),
                rates=self._make_rates(angular_speed)
            )

    def _linear_interpolation(self, t: float) -> Tuple[float, float]:
        """Linear sweep from min to max angle.

        Args:
            t: Elapsed time in seconds

        Returns:
            Tuple of (angle, angular_speed) in radians
        """
        progress = self.progress(t)
        angle = self.angle_min + (self.angle_max - self.angle_min) * progress

        # Constant rate during sweep, zero when complete
        if self.is_complete(t):
            angular_speed = 0.0
        else:
            angular_speed = (self.angle_max - self.angle_min) / self.duration

        return angle, angular_speed

    def _two_step_interpolation(self, t: float) -> Tuple[float, float]:
        """Three-phase interpolation: sweep up -> pause -> sweep down.

        Phase 1: Sweep from min to max
        Phase 2: Hold at max for pause_duration
        Phase 3: Sweep from max back to min

        Args:
            t: Elapsed time in seconds

        Returns:
            Tuple of (angle, angular_speed) in radians
        """
        pause_duration = 5.0  # 5-second pause at max pitch
        half_duration = (self.duration - pause_duration) / 2.0
        phase2_start = half_duration + pause_duration

        if t < half_duration:
            # Phase 1: Sweep from min to max
            progress = t / half_duration
            angle = self.angle_min + (self.angle_max - self.angle_min) * progress
            angular_speed = (self.angle_max - self.angle_min) / half_duration
        elif t < phase2_start:
            # Phase 2: Hold at max
            angle = self.angle_max
            angular_speed = 0.0
        elif t < self.duration:
            # Phase 3: Sweep from max to min
            progress = (t - phase2_start) / half_duration
            angle = self.angle_max + (self.angle_min - self.angle_max) * progress
            angular_speed = (self.angle_min - self.angle_max) / half_duration
        else:
            # Complete: hold at min
            angle = self.angle_min
            angular_speed = 0.0

        return angle, angular_speed

    def _make_orientation(self, angle: float) -> Dict[str, float]:
        """Create orientation dict with angle on specified axis.

        Args:
            angle: Angle in radians

        Returns:
            Orientation dict with roll, pitch, yaw (one set to angle, others zero)
        """
        orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        orientation[self.axis] = angle
        return orientation

    def _make_rates(self, angular_speed: float) -> Dict[str, float]:
        """Create rates dict with speed on specified axis.

        Args:
            angular_speed: Angular speed in rad/s

        Returns:
            Rates dict with rollspeed, pitchspeed, yawspeed
        """
        rates = {'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0}
        rates[f'{self.axis}speed'] = angular_speed
        return rates
