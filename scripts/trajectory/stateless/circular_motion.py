"""
Circular motion trajectory implementations.

Handles circular paths in different planes (XY horizontal, YZ vertical)
with optional pitch toward center orientation.

Trajectories: 5 (XY circle), 11 (YZ circle with pitch toward center)
"""

import math
from typing import Dict, Any, Tuple

from ..base import StatelessTrajectory


class CircularMotionTrajectory(StatelessTrajectory):
    """Circular motion with configurable plane and orientation.

    A parameterized trajectory class that handles circular paths in
    different planes. Instead of duplicating code for XY and YZ circles,
    this class accepts parameters to configure the behavior.

    Trajectories handled:
    - Trajectory 5: Circle in XY plane with yaw tangent to path
    - Trajectory 11: Circle in YZ plane with pitch facing center

    Example:
        # Trajectory 5: XY horizontal circle
        traj = CircularMotionTrajectory(config, plane='xy', radius=0.5)

        # Trajectory 11: YZ vertical circle with pitch toward center
        traj = CircularMotionTrajectory(config, plane='yz', radius=0.2, pitch_toward_center=True)
    """

    def __init__(self, config, plane: str = 'xy', radius: float = 0.5,
                 pitch_toward_center: bool = False, publisher=None):
        """Initialize circular motion trajectory.

        Args:
            config: SetpointConfig with TRAJECTORY_ANGLES parameters
            plane: 'xy' for horizontal circle, 'yz' for vertical circle
            radius: Circle radius in meters
            pitch_toward_center: If True, pitch always faces circle center (for YZ)
            publisher: Optional MAVROSFullStatePublisher
        """
        super().__init__(config, publisher)
        self.plane = plane
        self.radius = radius
        self.pitch_toward_center = pitch_toward_center

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute circular motion setpoint at time t.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, acceleration, orientation, rates
        """
        # Clamp time for final position hold
        t_clamped = min(t, self.duration)
        active = not self.is_complete(t)

        # Angular velocity for the circular path
        omega = 2 * math.pi / self.duration

        if self.plane == 'xy':
            angle = 2 * math.pi * t_clamped / self.duration
            pos, vel, acc = self._xy_motion(angle, omega, active)
            yaw = angle  # Point tangent to circle
            pitch = -0.523599  # Fixed pitch from original (30 degrees)
            yawspeed = omega if active else 0.0
            pitchspeed = 0.0
        else:  # 'yz'
            # Start at front of circle (y=0-radius, z=center)
            angle = 2 * math.pi * t_clamped / self.duration - math.pi / 2.0
            pos, vel, acc = self._yz_motion(angle, omega, active)
            yaw = 0.0  # No yaw rotation in YZ plane
            # Pitch faces center when pitch_toward_center is True
            pitch = -(angle + math.pi / 2.0) if self.pitch_toward_center else 0.0
            yawspeed = 0.0
            pitchspeed = -omega if self.pitch_toward_center and active else 0.0

        return self.build_setpoint(
            position=pos,
            velocity=vel,
            acceleration=acc,
            orientation={'roll': 0.0, 'pitch': pitch, 'yaw': yaw},
            rates={'rollspeed': 0.0, 'pitchspeed': pitchspeed, 'yawspeed': yawspeed}
        )

    def _xy_motion(self, angle: float, omega: float, active: bool
                   ) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
        """Compute position, velocity, acceleration for XY circular motion.

        Circle in the horizontal XY plane:
        - X: cos(angle) * radius, offset so circle starts at origin
        - Y: sin(angle) * radius
        - Z: constant altitude

        Args:
            angle: Current angle in radians
            omega: Angular velocity in rad/s
            active: True if trajectory is still in motion

        Returns:
            Tuple of (position, velocity, acceleration) dicts
        """
        # Position: circular motion in XY plane
        # Offset center so path starts at x=0
        x = self.radius * math.cos(angle) - self.radius
        y = self.radius * math.sin(angle)
        z = self.config.TRAJECTORY_ANGLES['z']

        if active:
            # Tangential velocity
            vx = -self.radius * math.sin(angle) * omega
            vy = self.radius * math.cos(angle) * omega
            vz = 0.0

            # Centripetal acceleration
            ax = -self.radius * omega * omega * math.cos(angle)
            ay = -self.radius * omega * omega * math.sin(angle)
            az = 0.0
        else:
            vx = vy = vz = 0.0
            ax = ay = az = 0.0

        return (
            {'x': x, 'y': y, 'z': z},
            {'vx': vx, 'vy': vy, 'vz': vz},
            {'ax': ax, 'ay': ay, 'az': az}
        )

    def _yz_motion(self, angle: float, omega: float, active: bool
                   ) -> Tuple[Dict[str, float], Dict[str, float], Dict[str, float]]:
        """Compute position, velocity, acceleration for YZ circular motion.

        Circle in the vertical YZ plane:
        - X: constant (0)
        - Y: sin(angle) * radius
        - Z: cos(angle) * radius + center_z

        Starting point: front of circle (y=0-radius, z=center_z when angle=-pi/2)

        Args:
            angle: Current angle in radians (offset by -pi/2 for start position)
            omega: Angular velocity in rad/s
            active: True if trajectory is still in motion

        Returns:
            Tuple of (position, velocity, acceleration) dicts
        """
        center_z = self.config.TRAJECTORY_ANGLES['z']

        # Position: circular motion in YZ plane
        x = 0.0
        y = self.radius * math.sin(angle)
        z = center_z + self.radius * math.cos(angle)

        if active:
            # Tangential velocity in YZ plane
            vx = 0.0
            vy = self.radius * omega * math.cos(angle)
            vz = -self.radius * omega * math.sin(angle)

            # Centripetal acceleration in YZ plane
            ax = 0.0
            ay = -self.radius * omega * omega * math.sin(angle)
            az = -self.radius * omega * omega * math.cos(angle)
        else:
            vx = vy = vz = 0.0
            ax = ay = az = 0.0

        return (
            {'x': x, 'y': y, 'z': z},
            {'vx': vx, 'vy': vy, 'vz': vz},
            {'ax': ax, 'ay': ay, 'az': az}
        )
