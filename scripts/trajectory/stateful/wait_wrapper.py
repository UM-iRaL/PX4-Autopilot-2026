"""
WaitTrajectory wrapper implementation.

Wraps the WaitTrajectory class with state management and configuration API.

Trajectory: 10 (WaitTrajectory)
"""

import math
import time
from typing import Dict, Any, Optional, List, Tuple, Callable

import numpy as np
import rospy

from ..base import StatefulTrajectory
from ..wait_trajectory import WaitTrajectory

# Import waypoint helpers - handle both direct and package imports
try:
    from trajectory_loader import TrajectoryConfigLoader
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from trajectory_loader import TrajectoryConfigLoader


def _get_fallback_hover_waypoints() -> List[Tuple]:
    """Get fallback hover waypoints when loading fails."""
    zero_vel = np.array([0.0, 0.0, 0.0])
    zero_acc = np.array([0.0, 0.0, 0.0])
    zero_angles = np.array([0.0, 0.0, 0.0])
    zero_omega = np.array([0.0, 0.0, 0.0])
    zero_alpha = np.array([0.0, 0.0, 0.0])
    hover_point = np.array([0.0, 0.0, 0.7])
    return [
        (hover_point, zero_vel, zero_acc, zero_angles, zero_omega, zero_alpha, 0.0),
        (hover_point, zero_vel, zero_acc, zero_angles, zero_omega, zero_alpha, 10.0),
    ]


def _waypoints_selection(trajectory_type: str, config_loader=None) -> List[Tuple]:
    """Load waypoint trajectory from YAML configuration."""
    if config_loader is None:
        try:
            config_loader = TrajectoryConfigLoader()
        except Exception as e:
            rospy.logerr(f"Failed to create TrajectoryConfigLoader: {e}")
            return _get_fallback_hover_waypoints()

    try:
        waypoints = config_loader.get_waypoint_trajectory(trajectory_type)
        rospy.loginfo(f"Loaded trajectory '{trajectory_type}' from YAML config")
        return waypoints
    except Exception as e:
        rospy.logerr(f"Failed to load trajectory '{trajectory_type}': {e}")
        return _get_fallback_hover_waypoints()


class WaitTrajectoryWrapper(StatefulTrajectory):
    """Wrapper for WaitTrajectory with dynamic time-scaling.

    Provides a stateful interface to WaitTrajectory that handles:
    - Pre-configuration before trajectory start
    - Initialization timeout waiting for position feedback
    - Multiple initialization types (equations, quintic, piecewise)
    - State management and cleanup

    Must be configured via setup() or setup_from_waypoints() before
    starting trajectory 10.

    Trajectory handled: 10
    """

    def __init__(self, config, publisher=None, config_loader=None):
        """Initialize WaitTrajectory wrapper.

        Args:
            config: SetpointConfig with WAIT_TRAJECTORY parameters
            publisher: MAVROSFullStatePublisher for pose feedback
            config_loader: Optional TrajectoryConfigLoader for YAML waypoints
        """
        super().__init__(config, publisher)
        self._config_loader = config_loader
        self._wait_trajectory: Optional[WaitTrajectory] = None
        self._start_time: Optional[float] = None

        # Current position (set by manager from pose feedback)
        self.current_position: Optional[np.ndarray] = None

        # Pre-initialize with default trajectory
        self._pre_initialize_default()

    def _pre_initialize_default(self) -> None:
        """Pre-initialize with default trajectory from config."""
        wt_config = self.config.WAIT_TRAJECTORY
        default_type = wt_config.get('type_quintic', 'default')

        try:
            waypoints = _waypoints_selection(default_type, self._config_loader)

            # Get interpolation method from config (defaults to quintic_polynomial)
            interpolation = self._config_loader.get_waypoint_trajectory_interpolation(default_type)
            if interpolation == 'piecewise_linear':
                self._setup_piecewise_linear(waypoints)
            else:
                self._setup_quintic_polynomial(waypoints)

        except Exception as e:
            rospy.logerr(f"Failed to pre-initialize WaitTrajectory: {e}")

    @property
    def wait_trajectory(self) -> Optional[WaitTrajectory]:
        """Access underlying WaitTrajectory instance."""
        return self._wait_trajectory

    def on_start(self) -> bool:
        """Activate WaitTrajectory when trajectory starts.

        Returns:
            True if activated successfully, False if not initialized
        """
        if not self._initialized:
            rospy.logerr("WaitTrajectory not configured - call setup() first")
            return False

        self._active = True
        self._start_time = time.time()
        rospy.loginfo(f"WaitTrajectory activated with gain={self.config.WAIT_TRAJECTORY['time_scaling_gain']}")
        return True

    def on_stop(self) -> None:
        """Deactivate WaitTrajectory when trajectory stops."""
        self._active = False
        self._start_time = None
        rospy.loginfo("WaitTrajectory deactivated")

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute WaitTrajectory setpoint at time t.

        Handles initialization check, position feedback timeout, and
        delegates to WaitTrajectory for path following.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, acceleration, orientation, rates
        """
        # Check initialization
        if not self._initialized or self._wait_trajectory is None:
            rospy.logerr("WaitTrajectory not configured - call setup() first")
            return self._hover_setpoint()

        # Activate on first call if not already active
        if not self._active:
            self.on_start()

        wt_config = self.config.WAIT_TRAJECTORY

        # Wait for position feedback during initialization timeout
        if t < wt_config['initialization_timeout']:
            if self.current_position is None:
                rospy.loginfo_throttle(0.5,
                    f"Waiting for position ({t:.1f}/{wt_config['initialization_timeout']:.1f}s)...")
                return self._hover_setpoint()

        # Get state from WaitTrajectory
        try:
            x, v, a, theta, omega, alpha = self._wait_trajectory.get(t, robot_pos=self.current_position)
        except Exception as e:
            rospy.logerr(f"WaitTrajectory.get() failed: {e}")
            return self._hover_setpoint()

        return self.build_setpoint(
            position={'x': x[0], 'y': x[1], 'z': x[2]},
            velocity={'vx': v[0], 'vy': v[1], 'vz': v[2]},
            acceleration={'ax': a[0], 'ay': a[1], 'az': a[2]},
            orientation={'roll': theta[0], 'pitch': theta[1], 'yaw': theta[2]},
            rates={'rollspeed': omega[0], 'pitchspeed': omega[1], 'yawspeed': omega[2]}
        )

    def setup(self, init_type: str, time_scaling_gain: float = None, **kwargs) -> bool:
        """Configure WaitTrajectory with specific parameters.

        Public API for configuring the trajectory before starting.

        Args:
            init_type: 'equations', 'quintic_polynomial', or 'piecewise_linear'
            time_scaling_gain: Override default gain from config
            **kwargs: Parameters for WaitTrajectory initialization
                For 'equations': pos_func, vel_func, acc_func, angle_func, omega_func, alpha_func
                For 'quintic_polynomial': waypoints list
                For 'piecewise_linear': waypoints list

        Returns:
            True if setup succeeded, False otherwise
        """
        if time_scaling_gain is None:
            time_scaling_gain = self.config.WAIT_TRAJECTORY['time_scaling_gain']

        try:
            self._wait_trajectory = WaitTrajectory(
                init_type=init_type,
                time_scaling_gain=time_scaling_gain,
                **kwargs
            )
            self._initialized = True
            rospy.loginfo(f"WaitTrajectory configured: type={init_type}, gain={time_scaling_gain}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to initialize WaitTrajectory: {e}")
            self._initialized = False
            return False

    def setup_from_waypoints(self, trajectory_type: str,
                             interpolation: str = 'quintic_polynomial') -> bool:
        """Configure WaitTrajectory from YAML waypoints.

        Args:
            trajectory_type: Name of trajectory in YAML config
            interpolation: 'quintic_polynomial' or 'piecewise_linear'

        Returns:
            True if setup succeeded, False otherwise
        """
        waypoints = _waypoints_selection(trajectory_type, self._config_loader)

        if interpolation == 'piecewise_linear':
            return self._setup_piecewise_linear(waypoints)
        else:
            return self._setup_quintic_polynomial(waypoints)

    def setup_circular_equation(self) -> bool:
        """Configure circular trajectory using equation functions.

        Creates a circular path in XY plane:
        - 0.5m radius
        - Duration from config
        - Zero attitude
        """
        duration = self.config.TRAJECTORY_ANGLES['duration']
        radius = 0.5
        center_z = self.config.TRAJECTORY_ANGLES['z']
        omega = 2 * math.pi / duration

        # Position function
        def pos_func(t):
            return np.array([
                radius * math.cos(2 * math.pi * t / duration) - radius,
                radius * math.sin(2 * math.pi * t / duration),
                center_z
            ])

        # Velocity function
        def vel_func(t):
            return np.array([
                -radius * omega * math.sin(2 * math.pi * t / duration),
                radius * omega * math.cos(2 * math.pi * t / duration),
                0.0
            ])

        # Acceleration function
        def acc_func(t):
            return np.array([
                -radius * omega * omega * math.cos(2 * math.pi * t / duration),
                -radius * omega * omega * math.sin(2 * math.pi * t / duration),
                0.0
            ])

        # Zero orientation and rates
        def angle_func(t):
            return np.array([0.0, 0.0, 0.0])

        def omega_func(t):
            return np.array([0.0, 0.0, 0.0])

        def alpha_func(t):
            return np.array([0.0, 0.0, 0.0])

        return self.setup(
            'equations',
            pos_func=pos_func,
            vel_func=vel_func,
            acc_func=acc_func,
            angle_func=angle_func,
            omega_func=omega_func,
            alpha_func=alpha_func
        )

    def _setup_quintic_polynomial(self, waypoints: List[Tuple]) -> bool:
        """Setup quintic polynomial interpolation."""
        return self.setup('quintic_polynomial', waypoints=waypoints)

    def _setup_piecewise_linear(self, waypoints: List[Tuple]) -> bool:
        """Setup piecewise linear interpolation."""
        return self.setup('piecewise_linear', waypoints=waypoints)

    def _hover_setpoint(self) -> Dict[str, Any]:
        """Return safe hover setpoint."""
        return self.build_setpoint(
            position=self.hover_position(),
            velocity=self.zero_velocity(),
            orientation=self.level_orientation(),
            rates=self.zero_rates()
        )
