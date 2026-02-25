"""
Trajectory manager for predefined drone trajectories.

This module provides the TrajectoryManager class which orchestrates various
predefined trajectories. Individual trajectory implementations are in the
stateless/ and stateful/ subdirectories.

The manager handles:
- Trajectory lifecycle (start, stop, get_setpoint)
- State management (current trajectory, timing)
- Resource management (hand tracker, wait trajectory)
- Pose feedback distribution to stateful trajectories
"""

import time
import math
from typing import Dict, Any, Optional

import numpy as np
import rospy

# Import configuration
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.setpoint_config import SetpointConfig
from vision.hand_tracking import HandTracker
from trajectory_loader import TrajectoryConfigLoader
from utils.math_utils import quaternion_angular_distance, vector_magnitude

# Import base classes
from .base import BaseTrajectory

# Import trajectory implementations
from .stateless import (
    AngularSweepTrajectory,
    CircularMotionTrajectory,
    CombinedRotationTrajectory,
)
from .stateful import (
    TakeoffTrajectory,
    LandingTrajectory,
    EmergencyLandingTrajectory,
    HandTrackingTrajectory,
    WaitTrajectoryWrapper,
    KeyboardTeleopTrajectory,
)


class TrajectoryManager:
    """Manages predefined trajectories for showcasing drone capabilities.

    The TrajectoryManager is the central coordinator for trajectory execution.
    It maintains a registry of trajectory instances and handles the lifecycle
    of starting, running, and stopping trajectories.

    Usage:
        manager = TrajectoryManager(publisher)
        manager.start_trajectory(6)  # Start takeoff
        setpoint = manager.get_current_setpoint()  # Called in loop
        manager.stop_trajectory()

    Trajectory Registry:
        1: Roll Sweep
        2: Pitch Sweep (quaternion)
        3: Yaw Sweep
        4: Combined 3-Axis Rotation
        5: Circular Path with Rotation (XY)
        6: Takeoff with Acceleration Ramp
        7: Landing with Attitude Stabilization
        8: Pitch Two-Step (quaternion)
        9: Hand Tracking (vision-based)
        10: WaitTrajectory (configurable path)
        11: YZ Circle with Pitch toward Center
        12: Emergency Landing (Reactive Stabilization)
        13: Keyboard Teleop (Arrow keys=XY, ,/.=Z, a/d=Yaw)
    """

    def __init__(self, publisher=None, config_loader=None):
        """Initialize trajectory manager.

        Args:
            publisher: MAVROSFullStatePublisher instance for state access
            config_loader: Optional TrajectoryConfigLoader for YAML waypoints
        """
        self.publisher = publisher
        self._config_loader = config_loader

        # Get config from publisher or create new
        self.config = publisher.config if publisher else SetpointConfig(config_loader)

        # Pre-initialize hand tracker
        self._hand_tracker = self._init_hand_tracker()

        # Create trajectory instances
        self._trajectories: Dict[int, BaseTrajectory] = {
            1: AngularSweepTrajectory(self.config, axis='roll', publisher=publisher),
            2: AngularSweepTrajectory(self.config, axis='pitch', use_quaternion=True, use_current_position=True, publisher=publisher),
            3: AngularSweepTrajectory(self.config, axis='yaw', publisher=publisher),
            4: CombinedRotationTrajectory(self.config, publisher=publisher),
            5: CircularMotionTrajectory(self.config, plane='xy', radius=0.5, publisher=publisher),
            6: TakeoffTrajectory(self.config, publisher=publisher),
            7: LandingTrajectory(self.config, publisher=publisher),
            8: AngularSweepTrajectory(self.config, axis='pitch', two_step=True,
                                       use_quaternion=True, publisher=publisher),
            9: HandTrackingTrajectory(self.config, publisher=publisher,
                                       hand_tracker=self._hand_tracker),
            10: WaitTrajectoryWrapper(self.config, publisher=publisher,
                                       config_loader=config_loader),
            11: CircularMotionTrajectory(self.config, plane='yz', radius=0.2,
                                          pitch_toward_center=True, publisher=publisher),
            12: EmergencyLandingTrajectory(self.config, publisher=publisher, manager=self),
            13: KeyboardTeleopTrajectory(self.config, publisher=publisher),
        }

        # Trajectory names for display
        self._trajectory_names = {
            1: f"Roll Sweep ({self.config.TRAJECTORY_ANGLES['roll_min']:.0f}° to {self.config.TRAJECTORY_ANGLES['roll_max']:.0f}°)",
            2: f"Pitch Sweep ({self.config.TRAJECTORY_ANGLES['pitch_min']:.0f}° to {self.config.TRAJECTORY_ANGLES['pitch_max']:.0f}°) [Quaternion]",
            3: f"Yaw Sweep ({self.config.TRAJECTORY_ANGLES['yaw_min']:.0f}° to {self.config.TRAJECTORY_ANGLES['yaw_max']:.0f}°)",
            4: "Combined 3-Axis Rotation",
            5: "Circular Path with Rotation",
            6: f"Takeoff with Acceleration Ramp (0m to {abs(self.config.TRAJECTORY_ANGLES['z']):.1f}m)",
            7: f"Landing (Duration: {self.config.LANDING['duration']:.1f}s)",
            8: f"Pitch Two-Step ({self.config.TRAJECTORY_ANGLES['pitch_min']:.0f}° → {self.config.TRAJECTORY_ANGLES['pitch_max']:.0f}° → {self.config.TRAJECTORY_ANGLES['pitch_min']:.0f}°)",
            9: "Hand Tracking (Vision-Based Attitude Control)",
            10: "WaitTrajectory (Time-Scaled Path Following)",
            11: "Vertical Circle in YZ Plane with 360° Pitch Rotation",
            12: "Emergency Landing (Reactive Stabilization)",
            13: "Keyboard Teleop (Arrow keys=XY, ,/.=Z, a/d=Yaw)"
        }

        # State management
        self.current_trajectory: Optional[int] = None
        self.trajectory_start_time: Optional[float] = None
        self.trajectory_duration = 20.0  # Default duration
        self.takeoff_complete = False

        # Pose feedback storage (updated by external pose callback)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_position: Optional[np.ndarray] = None
        self.current_velocity: Optional[np.ndarray] = None
        self.current_angular_velocity: Optional[np.ndarray] = None  # [wx, wy, wz] in body frame
        self.current_orientation_quat: Optional[np.ndarray] = None  # [x, y, z, w] in ENU

        # Auto-land error monitoring state
        self.auto_land_triggered = False
        self.last_error_check_time = 0.0
        self.last_error_log_time = 0.0
        self.position_error = 0.0
        self.orientation_error = 0.0
        self.velocity_error = 0.0
        self.angular_velocity_error = 0.0

        # Store commanded state for error comparison
        self.commanded_position: Optional[np.ndarray] = None
        self.commanded_velocity: Optional[np.ndarray] = None
        self.commanded_angular_velocity: Optional[np.ndarray] = None  # [wx, wy, wz] in body frame
        self.commanded_orientation_quat: Optional[np.ndarray] = None  # [x, y, z, w] in NED

    def _init_hand_tracker(self) -> Optional[HandTracker]:
        """Pre-initialize hand tracking resources.

        Returns:
            HandTracker instance or None if disabled/failed
        """
        if not self.config.HAND_TRACKING.get('enabled', False):
            rospy.loginfo("Hand tracking disabled (HAND_TRACKING['enabled'] = False)")
            return None

        rospy.loginfo("Pre-initializing hand tracking resources...")
        try:
            tracker = HandTracker(
                hand_tracking_config=self.config.HAND_TRACKING,
                camera_K=self.config.K,
                camera_D=self.config.D
            )
            if tracker.initialize():
                tracker.active = False  # Inactive until trajectory 9
                rospy.loginfo("Hand tracking resources ready (inactive until trajectory 9)")
                return tracker
            else:
                rospy.logwarn("Hand tracking initialization failed")
                return None
        except Exception as e:
            rospy.logerr(f"Hand tracking init error: {e}")
            return None

    def get_trajectory_name(self, trajectory_id: int) -> str:
        """Get human-readable trajectory name.

        Args:
            trajectory_id: Trajectory ID (1-11)

        Returns:
            Trajectory name string or "Unknown"
        """
        return self._trajectory_names.get(trajectory_id, "Unknown")

    def start_trajectory(self, trajectory_id: int) -> bool:
        """Start a trajectory.

        Stops any currently running trajectory, then starts the new one.
        Calls the trajectory's on_start() hook for setup.

        Args:
            trajectory_id: Trajectory ID (1-11)

        Returns:
            True if trajectory started successfully, False otherwise
        """
        if trajectory_id not in self._trajectories:
            rospy.logwarn(f"Unknown trajectory ID: {trajectory_id}")
            return False

        # Block non-takeoff/landing/emergency trajectories if takeoff hasn't completed
        if not self.takeoff_complete and trajectory_id not in (6, 7, 12):
            rospy.logwarn(
                f"Trajectory {trajectory_id} blocked: takeoff not complete. "
                "Only takeoff (6), landing (7), and emergency landing (12) are allowed before takeoff completes."
            )
            return False

        # Stop any active trajectory first (handles cleanup)
        if self.current_trajectory is not None:
            self.stop_trajectory()

        # Update pose feedback to stateful trajectories before starting
        self._update_stateful_pose_feedback()

        # Start new trajectory
        self.current_trajectory = trajectory_id
        self.trajectory_start_time = time.time()

        # Call trajectory's on_start hook - may return False to abort
        if not self._trajectories[trajectory_id].on_start():
            # Trajectory aborted (e.g., takeoff blocked because already airborne)
            self.current_trajectory = None
            self.trajectory_start_time = None
            return False

        # Reset takeoff flag and auto-land state for takeoff trajectory
        if trajectory_id == 6:
            self.takeoff_complete = False
            self.auto_land_triggered = False
            self.position_error = 0.0
            self.orientation_error = 0.0
            self.velocity_error = 0.0
            self.angular_velocity_error = 0.0

        rospy.loginfo(f"Starting trajectory {trajectory_id}: {self._trajectory_names[trajectory_id]}")
        return True

    def stop_trajectory(self) -> None:
        """Stop the current trajectory.

        Sets current_trajectory to None first to prevent race conditions,
        then calls the trajectory's on_stop() hook for cleanup.
        """
        if self.current_trajectory is None:
            return

        stopping_id = self.current_trajectory
        rospy.loginfo(f"Stopping trajectory {stopping_id}")

        # Set to None FIRST to prevent race conditions
        self.current_trajectory = None
        self.trajectory_start_time = None

        # Call trajectory's on_stop hook
        self._trajectories[stopping_id].on_stop()

        rospy.loginfo("Trajectory stopped successfully")

    def is_active(self) -> bool:
        """Check if a trajectory is currently active.

        Returns:
            True if a trajectory is running
        """
        return self.current_trajectory is not None

    def get_current_setpoint(self) -> Optional[Dict[str, Any]]:
        """Get the current setpoint based on active trajectory.

        Updates pose feedback to stateful trajectories, calculates elapsed
        time, and delegates to the current trajectory's get_setpoint().

        Returns:
            Setpoint dictionary or None if no trajectory active
        """
        if self.current_trajectory is None:
            return None

        # Update pose feedback for stateful trajectories
        self._update_stateful_pose_feedback()

        # Calculate elapsed time
        elapsed = time.time() - self.trajectory_start_time

        # Get setpoint from trajectory
        return self._trajectories[self.current_trajectory].get_setpoint(elapsed)

    def _update_stateful_pose_feedback(self) -> None:
        """Update pose feedback to stateful trajectories that need it."""
        # Landing trajectory needs current attitude
        landing = self._trajectories[7]
        if hasattr(landing, 'current_roll'):
            landing.current_roll = self.current_roll
            landing.current_pitch = self.current_pitch
            landing.current_yaw = self.current_yaw

        # Hand tracking trajectory needs current attitude
        hand_tracking = self._trajectories[9]
        if hasattr(hand_tracking, 'current_roll'):
            hand_tracking.current_roll = self.current_roll
            hand_tracking.current_pitch = self.current_pitch
            hand_tracking.current_yaw = self.current_yaw

        # WaitTrajectory wrapper needs current position
        wait_wrapper = self._trajectories[10]
        if hasattr(wait_wrapper, 'current_position'):
            wait_wrapper.current_position = self.current_position

    # ============================================================
    # Auto-Land Safety Monitoring
    # ============================================================

    def update_commanded_state(self, position: np.ndarray, velocity: np.ndarray,
                                orientation_quat: np.ndarray,
                                angular_velocity: np.ndarray = None) -> None:
        """Update commanded state for error monitoring.

        Called by the publisher after computing setpoints from trajectory.

        Args:
            position: Commanded position [x, y, z] in NED frame
            velocity: Commanded velocity [vx, vy, vz] in NED frame
            orientation_quat: Commanded quaternion [x, y, z, w] in NED aircraft frame
            angular_velocity: Commanded angular velocity [wx, wy, wz] in body frame (rad/s)
        """
        self.commanded_position = np.array(position)
        self.commanded_velocity = np.array(velocity)
        self.commanded_orientation_quat = np.array(orientation_quat)
        if angular_velocity is not None:
            self.commanded_angular_velocity = np.array(angular_velocity)

    def compute_position_error(self) -> float:
        """Compute position error between commanded and actual position.

        Returns:
            Position error magnitude in meters, or 0.0 if unavailable
        """
        if self.current_position is None or self.commanded_position is None:
            return 0.0

        error = np.linalg.norm(self.current_position - self.commanded_position)
        return float(error)

    def compute_orientation_error(self) -> float:
        """Compute orientation error using quaternion angular distance.

        Uses quaternion distance formula: θ = 2 * arccos(|q1 · q2|)
        This avoids gimbal lock issues with Euler angle comparisons.

        Returns:
            Angular distance in radians, or 0.0 if unavailable
        """
        if self.current_orientation_quat is None or self.commanded_orientation_quat is None:
            return 0.0

        try:
            error = quaternion_angular_distance(
                self.current_orientation_quat,
                self.commanded_orientation_quat
            )
            return float(error)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Orientation error calculation failed: {e}")
            return 0.0

    def compute_velocity_error(self) -> float:
        """Compute velocity error between commanded and actual velocity.

        Returns:
            Velocity error magnitude in m/s, or 0.0 if unavailable
        """
        if self.current_velocity is None or self.commanded_velocity is None:
            return 0.0

        error = np.linalg.norm(self.current_velocity - self.commanded_velocity)
        return float(error)

    def compute_angular_velocity_error(self) -> float:
        """Compute angular velocity error between commanded and actual angular velocity.

        Important for omnidirectional drones where angular rate control is critical
        for stability during tilted flight.

        Returns:
            Angular velocity error magnitude in rad/s, or 0.0 if unavailable
        """
        if self.current_angular_velocity is None or self.commanded_angular_velocity is None:
            return 0.0

        error = np.linalg.norm(self.current_angular_velocity - self.commanded_angular_velocity)
        return float(error)

    def check_and_trigger_auto_land(self) -> bool:
        """Check trajectory errors and trigger landing if thresholds exceeded.

        This is the main auto-land safety check. It computes position, orientation,
        and velocity errors, compares them against configured thresholds, and
        triggers the landing trajectory (7) if errors exceed safe limits.

        Returns:
            True if auto-land was triggered, False otherwise
        """
        auto_land_config = self.config.AUTO_LAND

        # Skip if disabled
        if not auto_land_config.get('enabled', False):
            return False

        # Skip if already triggered or landing
        if self.auto_land_triggered:
            return False

        # Skip if no trajectory is active
        if self.current_trajectory is None:
            return False

        # Skip if takeoff not complete (if required)
        if auto_land_config.get('require_takeoff_complete', True) and not self.takeoff_complete:
            return False

        # Skip for excluded trajectories (takeoff, landing, emergency landing)
        # Emergency landing (12) is always excluded — it's the safety trajectory
        # that auto-land itself targets, so re-triggering would reset rate damping.
        excluded = auto_land_config.get('excluded_trajectories', [6, 7])
        if self.current_trajectory in excluded or self.current_trajectory == 12:
            return False

        # Debounce: check minimum interval between checks
        now = time.time()
        min_interval = auto_land_config.get('min_check_interval', 0.1)
        if now - self.last_error_check_time < min_interval:
            return False
        self.last_error_check_time = now

        # Compute all errors
        self.position_error = self.compute_position_error()
        self.orientation_error = self.compute_orientation_error()
        self.velocity_error = self.compute_velocity_error()
        self.angular_velocity_error = self.compute_angular_velocity_error()

        # Get thresholds
        pos_threshold = auto_land_config.get('position_error_threshold', 1.5)
        orient_threshold = auto_land_config.get('orientation_error_threshold', 1.0)
        vel_threshold = auto_land_config.get('velocity_error_threshold', 2.0)
        ang_vel_threshold = auto_land_config.get('angular_velocity_error_threshold', 1.5)

        # Check which errors exceed thresholds
        pos_exceeded = self.position_error > pos_threshold
        orient_exceeded = self.orientation_error > orient_threshold
        vel_exceeded = self.velocity_error > vel_threshold
        ang_vel_exceeded = self.angular_velocity_error > ang_vel_threshold

        # Determine if we should trigger based on configuration
        trigger_on_any = auto_land_config.get('trigger_on_any_error', True)

        should_trigger = False
        trigger_reasons = []

        if trigger_on_any:
            # Trigger if ANY error exceeds threshold
            if pos_exceeded:
                should_trigger = True
                trigger_reasons.append(f"position={self.position_error:.2f}m > {pos_threshold:.2f}m")
            if orient_exceeded:
                should_trigger = True
                trigger_reasons.append(f"orientation={math.degrees(self.orientation_error):.1f}° > {math.degrees(orient_threshold):.1f}°")
            if vel_exceeded:
                should_trigger = True
                trigger_reasons.append(f"velocity={self.velocity_error:.2f}m/s > {vel_threshold:.2f}m/s")
            if ang_vel_exceeded:
                should_trigger = True
                trigger_reasons.append(f"angular_velocity={math.degrees(self.angular_velocity_error):.1f}°/s > {math.degrees(ang_vel_threshold):.1f}°/s")
        else:
            # Trigger only if ALL errors exceed thresholds
            if pos_exceeded and orient_exceeded and vel_exceeded and ang_vel_exceeded:
                should_trigger = True
                trigger_reasons = [
                    f"position={self.position_error:.2f}m",
                    f"orientation={math.degrees(self.orientation_error):.1f}°",
                    f"velocity={self.velocity_error:.2f}m/s",
                    f"angular_velocity={math.degrees(self.angular_velocity_error):.1f}°/s"
                ]

        # Log errors periodically even if not triggering
        log_interval = auto_land_config.get('log_interval', 5.0)
        if log_interval > 0 and now - self.last_error_log_time >= log_interval:
            self.last_error_log_time = now
            rospy.loginfo(
                f"Trajectory errors - pos: {self.position_error:.3f}m, "
                f"orient: {math.degrees(self.orientation_error):.1f}°, "
                f"vel: {self.velocity_error:.3f}m/s, "
                f"ang_vel: {math.degrees(self.angular_velocity_error):.1f}°/s"
            )

        # Trigger auto-land if needed
        if should_trigger:
            # Get target trajectory from config (12 = emergency landing, 7 = normal landing)
            target_traj = auto_land_config.get('target_trajectory', 12)
            traj_name = self._trajectory_names.get(target_traj, f"Trajectory {target_traj}")

            rospy.logwarn("=" * 60)
            rospy.logwarn("AUTO-LAND TRIGGERED!")
            rospy.logwarn(f"Trajectory {self.current_trajectory} errors exceeded thresholds:")
            for reason in trigger_reasons:
                rospy.logwarn(f"  - {reason}")
            rospy.logwarn(f"Switching to {traj_name}")
            rospy.logwarn("=" * 60)

            self.auto_land_triggered = True
            self.start_trajectory(target_traj)
            return True

        return False

    # ============================================================
    # Backward compatibility properties and methods
    # ============================================================

    @property
    def hand_tracker(self) -> Optional[HandTracker]:
        """Access hand tracker for backward compatibility."""
        return self._hand_tracker

    @property
    def wait_trajectory(self):
        """Access WaitTrajectory instance for backward compatibility."""
        wrapper = self._trajectories.get(10)
        return wrapper.wait_trajectory if wrapper else None

    @property
    def wait_trajectory_initialized(self) -> bool:
        """Check if WaitTrajectory is initialized."""
        wrapper = self._trajectories.get(10)
        return wrapper.is_initialized if wrapper else False

    @property
    def wait_trajectory_active(self) -> bool:
        """Check if WaitTrajectory is active."""
        wrapper = self._trajectories.get(10)
        return wrapper.is_active if wrapper else False

    @property
    def hand_tracking_start_time(self) -> Optional[float]:
        """Get hand tracking start time for backward compatibility."""
        ht = self._trajectories.get(9)
        return ht._start_time if ht else None

    @property
    def trajectories(self) -> Dict[int, BaseTrajectory]:
        """Access trajectory registry for backward compatibility."""
        return self._trajectories

    @property
    def trajectory_names(self) -> Dict[int, str]:
        """Access trajectory names for backward compatibility."""
        return self._trajectory_names

    def setup_wait_trajectory(self, init_type: str, time_scaling_gain: float = None,
                               **kwargs) -> bool:
        """Configure WaitTrajectory before starting trajectory 10.

        Public API for configuring the WaitTrajectory path.

        Args:
            init_type: 'equations', 'quintic_polynomial', or 'piecewise_linear'
            time_scaling_gain: Override default gain (optional)
            **kwargs: Path definition parameters

        Returns:
            True if setup succeeded

        Example:
            # Circular path using equations
            manager.setup_wait_trajectory(
                'equations',
                pos_func=lambda t: np.array([...]),
                vel_func=lambda t: np.array([...]),
                ...
            )

            # Waypoint trajectory
            manager.setup_wait_trajectory(
                'quintic_polynomial',
                waypoints=waypoints
            )
        """
        wrapper = self._trajectories.get(10)
        if wrapper is None:
            rospy.logerr("WaitTrajectory wrapper not found")
            return False
        return wrapper.setup(init_type, time_scaling_gain, **kwargs)

    # Legacy cleanup methods for backward compatibility
    def _cleanup_hand_tracking(self) -> None:
        """Cleanup hand tracking (delegated to trajectory)."""
        ht = self._trajectories.get(9)
        if ht:
            ht.on_stop()

    def _cleanup_wait_trajectory(self) -> None:
        """Cleanup WaitTrajectory (delegated to trajectory)."""
        wrapper = self._trajectories.get(10)
        if wrapper:
            wrapper.on_stop()

    # Legacy setup methods for backward compatibility
    def _setup_wait_trajectory_quintic_polynomial(self, waypoints) -> bool:
        """Setup quintic polynomial trajectory."""
        wrapper = self._trajectories.get(10)
        return wrapper._setup_quintic_polynomial(waypoints) if wrapper else False

    def _setup_wait_trajectory_piecewise_linear(self, waypoints) -> bool:
        """Setup piecewise linear trajectory."""
        wrapper = self._trajectories.get(10)
        return wrapper._setup_piecewise_linear(waypoints) if wrapper else False
