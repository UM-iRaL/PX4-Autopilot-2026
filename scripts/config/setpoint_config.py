"""
Configuration class for MAVROS setpoints.

This module provides the SetpointConfig class which loads trajectory and control
configurations from YAML files using the TrajectoryConfigLoader.
"""

import numpy as np
import rospy
from trajectory_loader import TrajectoryConfigLoader, ConfigValidationError


class SetpointConfig:
    """Configuration class for setpoints - loaded from YAML configuration file"""

    def __init__(self, config_loader=None):
        """
        Initialize configuration from YAML file

        Args:
            config_loader: Optional TrajectoryConfigLoader instance.
                          If None, creates a new loader with default config path.

        Raises:
            ConfigValidationError: If configuration file is invalid or cannot be loaded
        """
        if config_loader is None:
            config_loader = TrajectoryConfigLoader()

        self._loader = config_loader
        self._load_from_yaml()

    def _load_from_yaml(self):
        """Load configuration from YAML using the config loader"""
        # Initial setpoints
        self.POSITION = self._loader.get_initial_position()
        self.VELOCITY = self._loader.get_initial_velocity()
        self.ACCELERATION = self._loader.get_initial_acceleration()
        self.ORIENTATION = self._loader.get_initial_orientation()  # Already in radians
        self.ANGULAR_RATES = self._loader.get_initial_angular_rates()

        # Parametric trajectory configs
        self.TRAJECTORY_ANGLES = self._loader.get_trajectory_angles_config()

        # Takeoff and landing configs
        self.TAKEOFF = self._loader.get_parametric_config('takeoff')
        self.LANDING = self._loader.get_parametric_config('landing')

        # Hand tracking configuration
        self.HAND_TRACKING = self._loader.get_hand_tracking_config()

        # WaitTrajectory configuration
        self.WAIT_TRAJECTORY = self._loader.get_wait_trajectory_config()

        # Auto-land safety configuration
        self.AUTO_LAND = self._loader.get_auto_land_config()

        # Emergency landing configuration (reactive stabilization)
        self.EMERGENCY_LANDING = self._loader.get_emergency_landing_config()

        # Keyboard teleop configuration (manual keyboard control, trajectory 13)
        self.KEYBOARD_TELEOP = self._loader.get_keyboard_teleop_config()

        # Camera intrinsics
        self.K = self._loader.get_camera_intrinsics_matrix()
        self.D = self._loader.get_camera_distortion_coefficients()

        # Publishing rate and timeout
        self.PUBLISH_RATE_HZ = self._loader.get_publish_rate()
        self.TIMEOUT_SEC = self._loader.get_timeout()

        rospy.loginfo(f"Configuration loaded from: {self._loader.config_path}")
