"""
Hand tracking trajectory implementation.

Vision-based attitude control using camera feed and hand detection.

Trajectory: 9 (Hand Tracking)
"""

import time
from typing import Dict, Any, Optional

import numpy as np
import rospy

from ..base import StatefulTrajectory

# Import math utilities - handle both direct and package imports
try:
    from utils.math_utils import wrap_angle
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from utils.math_utils import wrap_angle


class HandTrackingTrajectory(StatefulTrajectory):
    """Vision-based attitude control from hand detection.

    Uses camera feed and MediaPipe hand detection to control drone attitude:
    - Hand horizontal position -> Yaw correction
    - Hand vertical position -> Pitch correction
    - Hand rotation angle -> Roll command

    The trajectory maintains desired attitude state and updates it based
    on hand tracking input. When no hand is detected, the last attitude
    is held.

    Requires a pre-initialized HandTracker instance to be provided.

    Trajectory handled: 9
    """

    def __init__(self, config, publisher=None, hand_tracker=None):
        """Initialize hand tracking trajectory.

        Args:
            config: SetpointConfig with HAND_TRACKING parameters
            publisher: MAVROSFullStatePublisher for pose feedback
            hand_tracker: Pre-initialized HandTracker instance
        """
        super().__init__(config, publisher)
        self.hand_tracker = hand_tracker

        # Desired attitude state
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0

        # Current attitude (set by manager from pose feedback)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Hand detection state
        self._hand_detected_logged = False
        self._start_time: Optional[float] = None

    def on_start(self) -> bool:
        """Activate hand tracker when trajectory starts.

        Returns:
            True if activated successfully, False if hand tracking not initialized
        """
        if self.hand_tracker is None or not self.hand_tracker.initialized:
            rospy.logerr("Hand tracking not initialized - cannot start trajectory 9")
            return False

        if not self.hand_tracker.active:
            self.hand_tracker.active = True
            self.hand_tracker.last_detection_time = time.time()
            self._hand_detected_logged = False
            self._start_time = time.time()

            # Initialize desired attitude to current attitude
            self.desired_roll = self.current_roll
            self.desired_pitch = self.current_pitch
            self.desired_yaw = self.current_yaw

            self._active = True
            rospy.loginfo("Hand tracking activated - show your hand to the camera")

        return True

    def on_stop(self) -> None:
        """Deactivate hand tracker when trajectory stops."""
        if self.hand_tracker is not None:
            self.hand_tracker.deactivate()

        # Reset state
        self._active = False
        self._hand_detected_logged = False
        self._start_time = None

        # Reset desired attitude
        self.desired_roll = 0.0
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0

        rospy.loginfo("Hand tracking deactivated")

    def get_setpoint(self, t: float) -> Dict[str, Any]:
        """Compute hand tracking setpoint at time t.

        Handles initialization timeout, hand detection, and attitude
        computation from hand position/orientation.

        Args:
            t: Elapsed time in seconds

        Returns:
            Setpoint dict with position, velocity, orientation, rates
        """
        # Check if hand tracker is available
        if self.hand_tracker is None or not self.hand_tracker.initialized:
            rospy.logerr_throttle(2.0, "Hand tracking not initialized - cannot run trajectory 9")
            return self._hover_setpoint()

        # Activate on first call if not already active
        if not self._active:
            self.on_start()
            if not self._active:
                return self._hover_setpoint()

        ht_config = self.config.HAND_TRACKING

        # Initialization period - wait for hand detection
        if t < ht_config['initialization_timeout'] and not self._hand_detected_logged:
            if self.hand_tracker.target_found:
                rospy.loginfo("Hand detected - starting tracking control")
                self._hand_detected_logged = True
            else:
                rospy.loginfo_throttle(0.5,
                    f"Waiting for hand detection ({t:.1f}/{ht_config['initialization_timeout']:.1f}s)...")
                return self._hover_setpoint()

        # Compute attitude from hand tracking
        if self.hand_tracker.target_found:
            self._update_attitude_from_hand(ht_config)
        else:
            # Hand lost - check timeout and hold last attitude
            time_since_detection = time.time() - self.hand_tracker.last_detection_time
            if time_since_detection > ht_config['target_loss_timeout']:
                rospy.logwarn_throttle(2.0,
                    f"Hand lost for {time_since_detection:.1f}s - holding last attitude")

        return self.build_setpoint(
            position=self.hover_position(),
            velocity=self.zero_velocity(),
            orientation={
                'roll': self.desired_roll,
                'pitch': self.desired_pitch,
                'yaw': self.desired_yaw
            },
            rates=self.zero_rates()
        )

    def _update_attitude_from_hand(self, ht_config: Dict) -> None:
        """Update desired attitude based on hand tracking input.

        Args:
            ht_config: HAND_TRACKING config dict with gains and limits
        """
        # Convert pixel error to angular error
        yaw_error, pitch_error = self.hand_tracker.pixel_to_angular_error(
            self.hand_tracker.target_pixel_x,
            self.hand_tracker.target_pixel_y
        )

        # Proportional control corrections
        yaw_correction = ht_config['yaw_gain'] * yaw_error
        pitch_correction = ht_config['pitch_gain'] * pitch_error

        # Update desired attitude using current attitude as baseline
        self.desired_pitch = self.current_pitch + pitch_correction
        self.desired_yaw = self.current_yaw + yaw_correction

        # Roll control from hand orientation (rotation angle)
        if abs(self.hand_tracker.hand_angle) > ht_config['hand_angle_deadzone']:
            self.desired_roll = -ht_config['roll_gain'] * self.hand_tracker.hand_angle
        else:
            self.desired_roll = 0.0

        # Clamp angles to limits
        self.desired_pitch = np.clip(self.desired_pitch,
                                      -ht_config['max_pitch'],
                                      ht_config['max_pitch'])
        self.desired_roll = np.clip(self.desired_roll,
                                     -ht_config['max_roll'],
                                     ht_config['max_roll'])
        self.desired_yaw = wrap_angle(self.desired_yaw)

    def _hover_setpoint(self) -> Dict[str, Any]:
        """Return safe hover setpoint (level attitude at hover altitude)."""
        return self.build_setpoint(
            position=self.hover_position(),
            velocity=self.zero_velocity(),
            orientation=self.level_orientation(),
            rates=self.zero_rates()
        )
