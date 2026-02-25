"""
Base classes for trajectory implementations.

This module provides abstract base classes that define the interface and
shared utilities for all trajectories:
- BaseTrajectory: Abstract base with common utilities (build_setpoint, quaternions)
- StatelessTrajectory: For pure time-based trajectories with no internal state
- StatefulTrajectory: For trajectories requiring initialization and cleanup
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import math


class BaseTrajectory(ABC):
    """Abstract base class for all trajectory implementations.

    All trajectories must implement get_setpoint(t) which returns a setpoint
    dictionary at elapsed time t. Trajectories can optionally override
    on_start() and on_stop() for lifecycle management.

    Provides shared utilities:
    - build_setpoint(): Standardized setpoint dict construction
    - make_quaternion_y_axis(): Quaternion for pitch rotation
    - hover_position(): Default hover position from config
    - zero_velocity(), zero_rates(): Zero dicts for common defaults
    """

    def __init__(self, config, publisher=None):
        """Initialize trajectory with configuration.

        Args:
            config: SetpointConfig instance with trajectory parameters
            publisher: Optional MAVROSFullStatePublisher for state access
        """
        self.config = config
        self.publisher = publisher

    @abstractmethod
    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute trajectory setpoint at elapsed time t.

        Args:
            t: Elapsed time in seconds since trajectory started

        Returns:
            Setpoint dictionary with keys:
            - 'position': {'x', 'y', 'z'}
            - 'velocity': {'vx', 'vy', 'vz'}
            - 'orientation': {'roll', 'pitch', 'yaw'} OR
              'orientation_quat': {'x', 'y', 'z', 'w'}
            - 'rates': {'rollspeed', 'pitchspeed', 'yawspeed'}
            - 'acceleration': {'ax', 'ay', 'az'} (optional)
            - Additional keys as needed (e.g., 'landing_complete')
        """
        pass

    def on_start(self) -> bool:
        """Called when trajectory starts. Override for setup logic.

        Returns:
            True if trajectory started successfully, False to abort
        """
        return True

    def on_stop(self) -> None:
        """Called when trajectory stops. Override for cleanup logic."""
        pass

    @staticmethod
    def build_setpoint(
        position: Dict[str, float],
        velocity: Dict[str, float],
        orientation: Dict[str, float] = None,
        orientation_quat: Dict[str, float] = None,
        rates: Dict[str, float] = None,
        acceleration: Dict[str, float] = None,
        **extras
    ) -> Dict[str, Any]:
        """Build standardized setpoint dictionary.

        Eliminates code duplication across all trajectory implementations
        by providing a single method to construct the return dictionary.

        Args:
            position: {'x', 'y', 'z'} in meters
            velocity: {'vx', 'vy', 'vz'} in m/s
            orientation: {'roll', 'pitch', 'yaw'} in radians (mutually exclusive with orientation_quat)
            orientation_quat: {'x', 'y', 'z', 'w'} quaternion (mutually exclusive with orientation)
            rates: {'rollspeed', 'pitchspeed', 'yawspeed'} in rad/s
            acceleration: {'ax', 'ay', 'az'} in m/s² (optional)
            **extras: Additional key-value pairs (e.g., landing_complete=True)

        Returns:
            Complete setpoint dictionary
        """
        result = {
            'position': position,
            'velocity': velocity,
            'rates': rates or {'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0}
        }
        if orientation is not None:
            result['orientation'] = orientation
        if orientation_quat is not None:
            result['orientation_quat'] = orientation_quat
        if acceleration is not None:
            result['acceleration'] = acceleration
        result.update(extras)
        return result

    @staticmethod
    def make_quaternion_y_axis(pitch: float) -> Dict[str, float]:
        """Create quaternion for pure Y-axis (pitch) rotation.

        For aerospace convention, pitch is rotation about Y-axis in body frame.
        q = [0, sin(θ/2), 0, cos(θ/2)] for rotation θ about Y-axis.

        Args:
            pitch: Pitch angle in radians

        Returns:
            Quaternion dict {'x', 'y', 'z', 'w'}
        """
        half = pitch / 2.0
        return {
            'x': 0.0,
            'y': math.sin(half),
            'z': 0.0,
            'w': math.cos(half)
        }

    def hover_position(self, z: float = None) -> Dict[str, float]:
        """Get default hover position from config.

        Args:
            z: Override z altitude (uses config default if None)

        Returns:
            Position dict {'x': 0, 'y': 0, 'z': altitude}
        """
        altitude = z if z is not None else self.config.TRAJECTORY_ANGLES['z']
        return {'x': 0.0, 'y': 0.0, 'z': altitude}

    @staticmethod
    def zero_velocity() -> Dict[str, float]:
        """Return zero velocity dict."""
        return {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}

    @staticmethod
    def zero_rates() -> Dict[str, float]:
        """Return zero angular rates dict."""
        return {'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0}

    @staticmethod
    def zero_acceleration() -> Dict[str, float]:
        """Return zero acceleration dict."""
        return {'ax': 0.0, 'ay': 0.0, 'az': 0.0}

    @staticmethod
    def level_orientation() -> Dict[str, float]:
        """Return level (zero) orientation dict."""
        return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    def hold_current_setpoint(self) -> Optional[Dict[str, Any]]:
        """Build a setpoint that holds the last commanded position and orientation.

        Reads the last commanded position and orientation from the publisher.
        Sets velocity, acceleration, and angular rates to zero.
        Uses stored quaternion when available to avoid gimbal lock at pitch=±90°.

        Returns:
            Hold setpoint dict, or None if publisher/position_target unavailable
        """
        if self.publisher is None:
            return None

        pt = getattr(self.publisher, 'position_target', None)
        if pt is None:
            return None

        position = {
            'x': pt.position.x,
            'y': pt.position.y,
            'z': pt.position.z
        }

        # Prefer stored quaternion to avoid gimbal lock at pitch=±90°
        q_ned = getattr(self.publisher, 'last_commanded_quat_ned', None)
        if q_ned is not None:
            return self.build_setpoint(
                position=position,
                velocity=self.zero_velocity(),
                acceleration=self.zero_acceleration(),
                orientation_quat={
                    'x': float(q_ned[0]),
                    'y': float(q_ned[1]),
                    'z': float(q_ned[2]),
                    'w': float(q_ned[3])
                },
                rates=self.zero_rates()
            )

        # Fallback to Euler angles
        roll = getattr(self.publisher, 'last_commanded_roll', 0.0)
        pitch = getattr(self.publisher, 'last_commanded_pitch', 0.0)
        yaw = getattr(self.publisher, 'last_commanded_yaw', pt.yaw)

        return self.build_setpoint(
            position=position,
            velocity=self.zero_velocity(),
            acceleration=self.zero_acceleration(),
            orientation={'roll': roll, 'pitch': pitch, 'yaw': yaw},
            rates=self.zero_rates()
        )


class StatelessTrajectory(BaseTrajectory):
    """Base class for trajectories that are pure functions of time.

    Stateless trajectories have no internal state - their output depends
    only on the elapsed time t. This makes them deterministic and easy
    to test. Examples: angular sweeps, circular paths.

    Provides:
    - duration: Trajectory duration from config
    - progress(t): Normalized progress [0, 1] clamped
    - is_complete(t): Check if trajectory time exceeded
    """

    @property
    def duration(self) -> float:
        """Trajectory duration in seconds from config."""
        return self.config.TRAJECTORY_ANGLES['duration']

    def progress(self, t: float) -> float:
        """Calculate normalized progress clamped to [0, 1].

        Eliminates duplicate min(t/duration, 1.0) calculations.

        Args:
            t: Elapsed time in seconds

        Returns:
            Progress value between 0.0 and 1.0
        """
        return min(t / self.duration, 1.0)

    def is_complete(self, t: float) -> bool:
        """Check if trajectory time has exceeded duration.

        Args:
            t: Elapsed time in seconds

        Returns:
            True if t >= duration
        """
        return t >= self.duration


class StatefulTrajectory(BaseTrajectory):
    """Base class for trajectories with internal state and lifecycle.

    Stateful trajectories may require initialization on first call,
    maintain state across calls, and need cleanup when stopped.
    Examples: landing (creates WaitTrajectory), hand tracking (activates tracker).

    Provides:
    - _initialized: Flag indicating first-call init completed
    - _active: Flag indicating trajectory is currently running
    - is_initialized, is_active: Property accessors
    """

    def __init__(self, config, publisher=None):
        super().__init__(config, publisher)
        self._initialized = False
        self._active = False

    @property
    def is_initialized(self) -> bool:
        """Check if trajectory has been initialized."""
        return self._initialized

    @property
    def is_active(self) -> bool:
        """Check if trajectory is currently active."""
        return self._active

    def reset(self) -> None:
        """Reset trajectory state for reuse.

        Override in subclasses to clear additional state.
        """
        self._initialized = False
        self._active = False
