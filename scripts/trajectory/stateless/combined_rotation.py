"""
Combined rotation trajectory implementation.

Handles simultaneous multi-axis rotation with different frequencies
on roll, pitch, and yaw axes.

Trajectory: 4 (Combined 3-Axis Rotation)
"""

import math
from typing import Dict, Any

from ..base import StatelessTrajectory


class CombinedRotationTrajectory(StatelessTrajectory):
    """Combined rotation on all three axes with different frequencies.

    Demonstrates simultaneous roll, pitch, and yaw control with
    sinusoidal oscillations at different frequencies. This creates
    a complex 3D rotation pattern.

    Roll:  0.8π amplitude, 1.0x base frequency
    Pitch: 0.6π amplitude, 1.5x base frequency
    Yaw:   1.0π amplitude, 0.5x base frequency

    Trajectory handled: 4
    """

    def __init__(self, config, publisher=None):
        """Initialize combined rotation trajectory.

        Args:
            config: SetpointConfig with TRAJECTORY_ANGLES parameters
            publisher: Optional MAVROSFullStatePublisher
        """
        super().__init__(config, publisher)

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute combined rotation setpoint at time t.

        Uses sinusoidal functions with different frequencies for each axis
        to create a complex rotation pattern.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, orientation, rates
        """
        duration = self.duration

        # Base angular frequency
        base_freq = 2 * math.pi / duration

        # Roll: 0.8π amplitude, 1.0x frequency
        roll = math.pi * 0.8 * math.sin(base_freq * t)
        rollspeed = math.pi * 0.8 * base_freq * math.cos(base_freq * t)

        # Pitch: 0.6π amplitude, 1.5x frequency
        pitch = math.pi * 0.6 * math.sin(base_freq * 1.5 * t)
        pitchspeed = math.pi * 0.6 * (base_freq * 1.5) * math.cos(base_freq * 1.5 * t)

        # Yaw: 1.0π amplitude, 0.5x frequency
        yaw = math.pi * math.sin(base_freq * 0.5 * t)
        yawspeed = math.pi * (base_freq * 0.5) * math.cos(base_freq * 0.5 * t)

        return self.build_setpoint(
            position=self.hover_position(),
            velocity=self.zero_velocity(),
            orientation={'roll': roll, 'pitch': pitch, 'yaw': yaw},
            rates={'rollspeed': rollspeed, 'pitchspeed': pitchspeed, 'yawspeed': yawspeed}
        )
