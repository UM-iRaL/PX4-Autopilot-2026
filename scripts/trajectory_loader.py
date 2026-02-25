#!/usr/bin/env python3
"""
Trajectory Configuration Loader

Loads and validates trajectory configurations from YAML files.
Provides typed access to configuration values with automatic unit conversions.

Author: IRAL Lab
Date: 2025
"""

import yaml
import numpy as np
import math
import os
from typing import Dict, List, Tuple, Any, Optional


class ConfigValidationError(Exception):
    """Raised when configuration validation fails"""
    pass


class TrajectoryConfigLoader:
    """
    Loads and validates trajectory configuration from YAML file

    Provides methods to access configuration sections with automatic type conversion
    and validation. Angles are automatically converted from degrees (YAML) to radians (Python).
    """

    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize loader

        Args:
            config_path: Path to YAML file. If None, uses default path
                        (config/trajectory_config.yaml relative to this file)

        Raises:
            ConfigValidationError: If config file not found or invalid
        """
        if config_path is None:
            # Default: config/trajectory_config.yaml relative to this file
            script_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(script_dir, 'config', 'trajectory_config.yaml')

        self.config_path = config_path
        self.config = self._load_config()
        self._validate_config()

        # Cache for loaded waypoint trajectories
        self._waypoint_cache: Dict[str, List[Tuple]] = {}

    def _load_config(self) -> Dict:
        """
        Load YAML configuration file

        Returns:
            Dictionary with configuration data

        Raises:
            ConfigValidationError: If file not found or invalid YAML
        """
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            raise ConfigValidationError(
                f"Configuration file not found: {self.config_path}\n"
                f"Please create config file from template."
            )
        except yaml.YAMLError as e:
            raise ConfigValidationError(
                f"Invalid YAML syntax in {self.config_path}: {e}"
            )

    def _validate_config(self) -> None:
        """
        Validate configuration structure and required fields

        Raises:
            ConfigValidationError: If required fields missing or invalid
        """
        # Check version
        if 'version' not in self.config:
            raise ConfigValidationError("Missing 'version' field in config")

        # Check required top-level sections
        required_sections = [
            'initial_setpoints',
            'parametric_trajectories',
            'waypoint_trajectories',
            'hand_tracking',
            'wait_trajectory'
        ]

        for section in required_sections:
            if section not in self.config:
                raise ConfigValidationError(
                    f"Missing required section '{section}' in config file"
                )

        # Validate initial_setpoints structure
        initial = self.config['initial_setpoints']
        required_initial = ['position', 'velocity', 'acceleration', 'orientation', 'angular_rates']
        for subsection in required_initial:
            if subsection not in initial:
                raise ConfigValidationError(
                    f"Missing '{subsection}' in initial_setpoints section"
                )

        # Validate parametric_trajectories structure
        parametric = self.config['parametric_trajectories']
        required_parametric = ['altitude', 'duration']
        for field in required_parametric:
            if field not in parametric:
                raise ConfigValidationError(
                    f"Missing '{field}' in parametric_trajectories section"
                )

    # ==========================================================================
    # General Configuration
    # ==========================================================================

    def get_publish_rate(self) -> int:
        """
        Get publishing rate in Hz

        Returns:
            Publishing rate (Hz)
        """
        return self.config.get('publish_rate_hz', 20)

    def get_timeout(self) -> int:
        """
        Get timeout for mode changes and arming

        Returns:
            Timeout (seconds)
        """
        return self.config.get('timeout_sec', 5)

    # ==========================================================================
    # Initial Setpoints
    # ==========================================================================

    def get_initial_position(self) -> Dict[str, float]:
        """
        Get initial position setpoints

        Returns:
            Dictionary with x, y, z keys (meters)
        """
        return self.config['initial_setpoints']['position'].copy()

    def get_initial_velocity(self) -> Dict[str, float]:
        """
        Get initial velocity setpoints

        Returns:
            Dictionary with vx, vy, vz keys (m/s)
        """
        return self.config['initial_setpoints']['velocity'].copy()

    def get_initial_acceleration(self) -> Dict[str, float]:
        """
        Get initial acceleration setpoints

        Returns:
            Dictionary with ax, ay, az keys (m/s^2)
        """
        return self.config['initial_setpoints']['acceleration'].copy()

    def get_initial_orientation(self) -> Dict[str, float]:
        """
        Get initial orientation (converted to radians)

        Returns:
            Dictionary with roll, pitch, yaw keys (RADIANS)
            Note: Values in YAML are in degrees, converted to radians here
        """
        orientation_deg = self.config['initial_setpoints']['orientation']
        return {
            'roll': math.radians(orientation_deg['roll']),
            'pitch': math.radians(orientation_deg['pitch']),
            'yaw': math.radians(orientation_deg['yaw'])
        }

    def get_initial_angular_rates(self) -> Dict[str, float]:
        """
        Get initial angular rates

        Returns:
            Dictionary with rollspeed, pitchspeed, yawspeed keys (rad/s)
        """
        return self.config['initial_setpoints']['angular_rates'].copy()

    # ==========================================================================
    # Parametric Trajectory Configuration
    # ==========================================================================

    def get_parametric_config(self, trajectory_name: str) -> Dict[str, Any]:
        """
        Get configuration for parametric trajectory

        Args:
            trajectory_name: Name of trajectory ('common', 'roll_sweep', 'pitch_sweep',
                           'yaw_sweep', 'circular_path', 'takeoff', 'landing')

        Returns:
            Dictionary with trajectory-specific parameters

        Raises:
            KeyError: If trajectory_name not found
        """
        parametric = self.config['parametric_trajectories']

        if trajectory_name == 'common':
            return {
                'altitude': parametric['altitude'],
                'duration': parametric['duration']
            }
        elif trajectory_name in parametric:
            return parametric[trajectory_name].copy()
        else:
            raise KeyError(f"Parametric trajectory '{trajectory_name}' not found in config")

    def get_trajectory_angles_config(self) -> Dict[str, float]:
        """
        Get TRAJECTORY_ANGLES configuration (for backward compatibility)

        Returns:
            Dictionary with roll_min, roll_max, pitch_min, pitch_max, yaw_min, yaw_max,
            z, duration keys (angles in DEGREES)
        """
        parametric = self.config['parametric_trajectories']

        return {
            'roll_min': parametric['roll_sweep']['min_angle'],
            'roll_max': parametric['roll_sweep']['max_angle'],
            'pitch_min': parametric['pitch_sweep']['min_angle'],
            'pitch_max': parametric['pitch_sweep']['max_angle'],
            'yaw_min': parametric['yaw_sweep']['min_angle'],
            'yaw_max': parametric['yaw_sweep']['max_angle'],
            'z': parametric['altitude'],
            'duration': parametric['duration']
        }

    # ==========================================================================
    # Waypoint Trajectories
    # ==========================================================================

    def get_waypoint_trajectory(self, trajectory_name: str) -> List[Tuple]:
        """
        Get waypoint trajectory as list of tuples

        Args:
            trajectory_name: Name of trajectory (e.g., 'square', 'whiteboard')

        Returns:
            List of waypoints as tuples: (pos, vel, acc, angles, omega, alpha, time, l1_params)
            where:
            - pos: np.array([x, y, z])
            - vel: np.array([vx, vy, vz])
            - acc: np.array([ax, ay, az])
            - angles: np.array([roll, pitch, yaw]) in RADIANS
            - omega: np.array([wx, wy, wz])
            - alpha: np.array([alphax, alphay, alphaz])
            - time: float
            - l1_params: dict or None - Optional L1 adaptive control parameters
                        {'enable': int, 'enable_force': int, 'enable_moment': int}

        Raises:
            KeyError: If trajectory_name not found
        """
        # Check cache first
        if trajectory_name in self._waypoint_cache:
            return self._waypoint_cache[trajectory_name]

        # Load from config
        if trajectory_name not in self.config['waypoint_trajectories']:
            raise KeyError(f"Waypoint trajectory '{trajectory_name}' not found in config")

        traj_config = self.config['waypoint_trajectories'][trajectory_name]
        waypoints = []

        for wp in traj_config['waypoints']:
            if isinstance(wp, dict):
                # Dict format
                pos = np.array(wp['position'], dtype=float)
                vel = np.array(wp['velocity'], dtype=float)
                acc = np.array(wp['acceleration'], dtype=float)

                # Convert orientation from degrees to radians
                angles_deg = wp['orientation']
                angles = np.array([
                    math.radians(angles_deg[0]),  # roll
                    math.radians(angles_deg[1]),  # pitch
                    math.radians(angles_deg[2])   # yaw
                ], dtype=float)

                omega = np.array(wp['angular_velocity'], dtype=float)
                alpha = np.array(wp['angular_acceleration'], dtype=float)
                time = float(wp['time'])

                # Extract optional L1 adaptive control parameters
                l1_params = None
                if 'l1_params' in wp:
                    l1_params = {
                        'enable': int(wp['l1_params'].get('enable', 0)),
                        'enable_force': int(wp['l1_params'].get('enable_force', 0)),
                        'enable_moment': int(wp['l1_params'].get('enable_moment', 0))
                    }

                # Extract optional decouple flags for feedforward-only fields
                decouple_flags = None
                if 'decouple' in wp:
                    decouple_flags = {
                        'acceleration': bool(wp['decouple'].get('acceleration', False)),
                        'velocity': bool(wp['decouple'].get('velocity', False)),
                    }

            else:
                # Array format: [x,y,z,vx,vy,vz,ax,ay,az,r,p,y,wx,wy,wz,alphax,alphay,alphaz,t]
                pos = np.array(wp[0:3], dtype=float)
                vel = np.array(wp[3:6], dtype=float)
                acc = np.array(wp[6:9], dtype=float)

                # Convert orientation from degrees to radians
                angles = np.array([
                    math.radians(wp[9]),   # roll
                    math.radians(wp[10]),  # pitch
                    math.radians(wp[11])   # yaw
                ], dtype=float)

                omega = np.array(wp[12:15], dtype=float)
                alpha = np.array(wp[15:18], dtype=float)
                time = float(wp[18])

                # Array format doesn't support L1 params or decouple flags
                l1_params = None
                decouple_flags = None

            # Validate waypoint
            self._validate_waypoint(pos, vel, acc, angles, omega, alpha, time)

            waypoints.append((pos, vel, acc, angles, omega, alpha, time, l1_params, decouple_flags))

        # Cache the result
        self._waypoint_cache[trajectory_name] = waypoints

        return waypoints

    def _validate_waypoint(self, pos, vel, acc, angles, omega, alpha, time):
        """
        Validate a single waypoint

        Args:
            pos: Position array
            vel: Velocity array
            acc: Acceleration array
            angles: Angles array (radians)
            omega: Angular velocity array
            alpha: Angular acceleration array
            time: Time value

        Raises:
            ConfigValidationError: If waypoint is invalid
        """
        # Check array shapes
        if pos.shape != (3,):
            raise ConfigValidationError(f"Position must have 3 elements, got {pos.shape}")
        if vel.shape != (3,):
            raise ConfigValidationError(f"Velocity must have 3 elements, got {vel.shape}")
        if acc.shape != (3,):
            raise ConfigValidationError(f"Acceleration must have 3 elements, got {acc.shape}")
        if angles.shape != (3,):
            raise ConfigValidationError(f"Angles must have 3 elements, got {angles.shape}")
        if omega.shape != (3,):
            raise ConfigValidationError(f"Angular velocity must have 3 elements, got {omega.shape}")
        if alpha.shape != (3,):
            raise ConfigValidationError(f"Angular acceleration must have 3 elements, got {alpha.shape}")

        # Check time is non-negative
        if time < 0:
            raise ConfigValidationError(f"Time must be non-negative, got {time}")

    def list_waypoint_trajectories(self) -> List[str]:
        """
        Get list of available waypoint trajectory names

        Returns:
            List of trajectory names
        """
        return list(self.config['waypoint_trajectories'].keys())

    def get_waypoint_trajectory_interpolation(self, trajectory_name: str) -> str:
        """
        Get interpolation method for a waypoint trajectory

        Args:
            trajectory_name: Name of trajectory (e.g., 'square', 'whiteboard')

        Returns:
            Interpolation method: 'quintic_polynomial' (default) or 'piecewise_linear'

        Raises:
            KeyError: If trajectory_name not found
        """
        if trajectory_name not in self.config['waypoint_trajectories']:
            raise KeyError(f"Waypoint trajectory '{trajectory_name}' not found in config")

        traj_config = self.config['waypoint_trajectories'][trajectory_name]
        return traj_config.get('interpolation', 'quintic_polynomial')

    # ==========================================================================
    # Hand Tracking Configuration
    # ==========================================================================

    def get_hand_tracking_config(self) -> Dict[str, Any]:
        """
        Get hand tracking configuration

        Returns:
            Dictionary with hand tracking parameters (flattened structure for HandTracker)
        """
        yaml_config = self.config['hand_tracking']

        # Create flattened config dictionary
        flat_config = {}

        # Add enabled flag
        flat_config['enabled'] = yaml_config['enabled']

        # Camera parameters (flatten camera section)
        camera = yaml_config['camera']
        flat_config['camera_topic'] = camera['topic']
        flat_config['image_width'] = camera['width']
        flat_config['image_height'] = camera['height']
        flat_config['center_x'] = camera['intrinsics']['cx']
        flat_config['center_y'] = camera['intrinsics']['cy']
        flat_config['focal_length_px'] = camera['intrinsics']['fx']
        flat_config['focal_length_py'] = camera['intrinsics']['fy']
        flat_config['fisheye_zoom'] = camera['fisheye_zoom']

        # Control parameters (flatten control section)
        control = yaml_config['control']
        flat_config['yaw_gain'] = control['yaw_gain']
        flat_config['pitch_gain'] = control['pitch_gain']
        flat_config['roll_gain'] = control['roll_gain']
        flat_config['max_pitch'] = math.radians(control['max_pitch_deg'])
        flat_config['max_roll'] = math.radians(control['max_roll_deg'])

        # Thresholds (flatten thresholds section)
        thresholds = yaml_config['thresholds']
        flat_config['pixel_deadzone_x'] = thresholds['pixel_deadzone_x']
        flat_config['pixel_deadzone_y'] = thresholds['pixel_deadzone_y']
        flat_config['hand_angle_deadzone'] = math.radians(thresholds['hand_angle_deadzone_deg'])
        flat_config['target_loss_timeout'] = thresholds['target_loss_timeout']
        flat_config['initialization_timeout'] = thresholds['initialization_timeout']

        return flat_config

    def get_camera_intrinsics_matrix(self) -> np.ndarray:
        """
        Get camera intrinsics matrix K

        Returns:
            3x3 numpy array with camera intrinsics
        """
        camera = self.config['hand_tracking']['camera']
        intrinsics = camera['intrinsics']

        K = np.array([
            [intrinsics['fx'], 0.0, intrinsics['cx']],
            [0.0, intrinsics['fy'], intrinsics['cy']],
            [0.0, 0.0, 1.0]
        ])

        return K

    def get_camera_distortion_coefficients(self) -> np.ndarray:
        """
        Get camera distortion coefficients D (Kannala-Brandt model)

        Returns:
            1D numpy array with distortion coefficients [k1, k2, k3, k4]
        """
        camera = self.config['hand_tracking']['camera']
        distortion = camera['distortion']

        D = np.array([
            distortion['k1'],
            distortion['k2'],
            distortion['k3'],
            distortion['k4']
        ])

        return D

    # ==========================================================================
    # WaitTrajectory Configuration
    # ==========================================================================

    def get_wait_trajectory_config(self) -> Dict[str, Any]:
        """
        Get WaitTrajectory configuration

        Returns:
            Dictionary with WaitTrajectory parameters
        """
        wait_config = self.config['wait_trajectory'].copy()

        # Rename 'default_trajectory' to 'type_quintic' for backward compatibility
        wait_config['type_quintic'] = wait_config['default_trajectory']

        return wait_config

    # ==========================================================================
    # Auto-Land Safety Configuration
    # ==========================================================================

    def get_auto_land_config(self) -> Dict[str, Any]:
        """
        Get auto-land safety configuration

        Returns:
            Dictionary with auto-land parameters:
            - enabled: bool - Enable/disable auto-land feature
            - position_error_threshold: float - Position error threshold (meters)
            - orientation_error_threshold: float - Quaternion angular distance threshold (radians)
            - velocity_error_threshold: float - Velocity error threshold (m/s)
            - angular_velocity_error_threshold: float - Angular velocity error threshold (rad/s)
            - min_check_interval: float - Minimum time between checks (seconds)
            - excluded_trajectories: List[int] - Trajectory IDs to exclude from monitoring
            - require_takeoff_complete: bool - Only check after takeoff
            - trigger_on_any_error: bool - Trigger on any single error vs all errors
            - log_interval: float - Interval for logging errors (seconds, 0 to disable)
        """
        # Default values if auto_land section doesn't exist
        defaults = {
            'enabled': False,
            'position_error_threshold': 1.5,
            'orientation_error_threshold': 1.0,
            'velocity_error_threshold': 2.0,
            'angular_velocity_error_threshold': 1.5,
            'min_check_interval': 0.1,
            'excluded_trajectories': [6, 7, 12],
            'require_takeoff_complete': True,
            'trigger_on_any_error': True,
            'log_interval': 5.0,
            'target_trajectory': 12  # 12 = emergency landing, 7 = normal landing
        }

        if 'auto_land' not in self.config:
            return defaults

        auto_land_config = self.config['auto_land'].copy()

        # Merge with defaults for any missing keys
        for key, value in defaults.items():
            if key not in auto_land_config:
                auto_land_config[key] = value

        return auto_land_config

    # ==========================================================================
    # Emergency Landing Configuration
    # ==========================================================================

    def get_emergency_landing_config(self) -> Dict[str, Any]:
        """
        Get emergency landing configuration for reactive stabilization

        Returns:
            Dictionary with emergency landing parameters:
            - rate_damping_threshold: float - Angular rate threshold to exit damping (rad/s)
            - max_rate_command: float - Maximum angular rate to command (rad/s)
            - max_attitude_rate: float - Maximum attitude correction rate (rad/s)
            - pitch_priority: bool - Correct pitch before roll
            - level_threshold: float - Attitude threshold for "level" (rad)
            - pitch_threshold: float - Pitch threshold to start roll correction (rad)
            - descent_rate: float - Descent speed (m/s)
            - ground_altitude: float - Target ground altitude (meters)
            - max_stabilization_time: float - Timeout for stabilization (seconds)
            - position_hold_gain: float - Gain for position hold
        """
        # Default values if emergency_landing section doesn't exist
        defaults = {
            'rate_damping_threshold': 0.3,
            'max_rate_command': 0.5,
            'max_attitude_rate': 0.2,
            'pitch_priority': True,
            'level_threshold': 0.1,
            'pitch_threshold': 0.15,
            'descent_rate': 0.3,
            'ground_altitude': -0.2,
            'max_stabilization_time': 15.0,
            'position_hold_gain': 0.5
        }

        if 'emergency_landing' not in self.config:
            return defaults

        emergency_config = self.config['emergency_landing'].copy()

        # Merge with defaults for any missing keys
        for key, value in defaults.items():
            if key not in emergency_config:
                emergency_config[key] = value

        return emergency_config

    def get_keyboard_teleop_config(self) -> Dict[str, Any]:
        """
        Get keyboard teleop configuration (trajectory 13).

        Returns:
            Dictionary with keyboard teleop parameters:
            - max_velocity_xy:   float - Maximum horizontal velocity (m/s)
            - max_velocity_z:    float - Maximum vertical velocity (m/s)
            - max_yaw_rate_deg:  float - Maximum yaw rate (deg/s)  [j/l keys]
            - max_pitch_rate_deg: float - Maximum pitch rate (deg/s) [i/k keys]
            - max_roll_rate_deg:  float - Maximum roll rate (deg/s)  [u/o keys]
            - max_pitch_deg:     float - Pitch clamp limit (deg)
            - max_roll_deg:      float - Roll clamp limit (deg)
        """
        defaults = {
            'max_velocity_xy': 0.3,
            'max_velocity_z': 0.2,
            'max_yaw_rate_deg': 20.0,
            'max_pitch_rate_deg': 20.0,
            'max_roll_rate_deg': 20.0,
            'max_pitch_deg': 90.0,
            'max_roll_deg': 90.0,
        }

        if 'keyboard_teleop' not in self.config:
            return defaults

        kb_config = self.config['keyboard_teleop'].copy()
        for key, value in defaults.items():
            if key not in kb_config:
                kb_config[key] = value

        return kb_config


if __name__ == '__main__':
    """Test the configuration loader"""
    import sys

    print("Testing TrajectoryConfigLoader...")
    print("=" * 70)

    try:
        # Load config
        config_path = sys.argv[1] if len(sys.argv) > 1 else None
        loader = TrajectoryConfigLoader(config_path)

        print(f"Configuration loaded from: {loader.config_path}")
        print(f"Version: {loader.config['version']}")
        print(f"Publish rate: {loader.get_publish_rate()} Hz")
        print()

        # Test initial setpoints
        print("Initial Setpoints:")
        print(f"  Position: {loader.get_initial_position()}")
        print(f"  Velocity: {loader.get_initial_velocity()}")
        print(f"  Orientation (radians): {loader.get_initial_orientation()}")
        print()

        # Test parametric configs
        print("Parametric Trajectories:")
        common = loader.get_parametric_config('common')
        print(f"  Altitude: {common['altitude']} m")
        print(f"  Duration: {common['duration']} s")

        roll_sweep = loader.get_parametric_config('roll_sweep')
        print(f"  Roll sweep: {roll_sweep['min_angle']}° to {roll_sweep['max_angle']}°")
        print()

        # Test waypoint trajectories
        print("Waypoint Trajectories:")
        trajectory_names = loader.list_waypoint_trajectories()
        print(f"  Available: {', '.join(trajectory_names)}")

        for name in trajectory_names:
            waypoints = loader.get_waypoint_trajectory(name)
            print(f"  {name}: {len(waypoints)} waypoints")
        print()

        # Test a specific trajectory
        print("Testing 'square' trajectory:")
        square_waypoints = loader.get_waypoint_trajectory('square')
        for i, (pos, vel, acc, angles, omega, alpha, time) in enumerate(square_waypoints):
            print(f"  Waypoint {i}: pos={pos}, time={time}s")
        print()

        print("All tests passed!")

    except ConfigValidationError as e:
        print(f"Configuration validation error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
