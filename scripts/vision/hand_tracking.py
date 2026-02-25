"""
Hand tracking module for vision-based attitude control.

This module provides the HandTracker class which uses MediaPipe and OpenCV
to detect hands in camera images and convert hand position/orientation
to attitude control commands.
"""

# Suppress TensorFlow/MediaPipe stdout pollution BEFORE any imports
# This must be at module level to take effect before TF is loaded
import os
import sys
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # Suppress TF C++ logging
os.environ['GLOG_minloglevel'] = '2'  # Suppress glog (MediaPipe)
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'  # Suppress oneDNN messages
os.environ['GRPC_VERBOSITY'] = 'ERROR'  # Suppress gRPC logging

# Suppress absl logging (used internally by MediaPipe)
try:
    import absl.logging
    absl.logging.set_verbosity(absl.logging.ERROR)
    absl.logging.set_stderrthreshold(absl.logging.ERROR)
except ImportError:
    pass

import math
import time
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Import math utilities for angle wrapping
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.math_utils import wrap_angle


class HandTracker:
    """
    Hand tracking class for vision-based drone attitude control.

    Uses MediaPipe hand detection to track hand position and orientation in camera images,
    converting pixel coordinates to angular errors for attitude control.
    """

    def __init__(self, hand_tracking_config=None, camera_K=None, camera_D=None):
        """
        Initialize hand tracker.

        Args:
            hand_tracking_config: Hand tracking configuration dict
            camera_K: Camera intrinsics matrix
            camera_D: Camera distortion coefficients
        """
        # Import SetpointConfig only as fallback
        if hand_tracking_config is None:
            from config.setpoint_config import SetpointConfig
            _config = SetpointConfig()
            hand_tracking_config = _config.HAND_TRACKING
            camera_K = _config.K
            camera_D = _config.D

        self.config = hand_tracking_config
        self.camera_K = camera_K
        self.camera_D = camera_D

        # MediaPipe and OpenCV objects
        self.mediapipe_hands = None
        self.mp_hands = None
        self.mp_drawing = None
        self.cv_bridge = None
        self.fisheye_map1 = None
        self.fisheye_map2 = None

        # ROS subscribers/publishers
        self.image_sub = None
        self.debug_image_pub = None

        # Tracking state
        self.target_found = False
        self.target_pixel_x = self.config['center_x']
        self.target_pixel_y = self.config['center_y']
        self.hand_angle = 0.0
        self.last_detection_time = None

        # Initialization flags
        self.initialized = False
        self.active = False

    def initialize(self):
        """
        Initialize MediaPipe and camera subscription for hand tracking.

        Returns:
            bool: True if initialization succeeded, False otherwise
        """
        if self.initialized:
            return True

        rospy.loginfo("Initializing hand tracking...")

        try:
            # Import MediaPipe (lazy import)
            # Note: TF/MediaPipe logging is suppressed at module level (top of file)
            import mediapipe as mp
            self.mp_hands = mp.solutions.hands
            self.mp_drawing = mp.solutions.drawing_utils

            # Create MediaPipe Hands detector
            self.mediapipe_hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.1
            )

            # Initialize CV Bridge
            self.cv_bridge = CvBridge()

            # Compute fisheye undistortion maps
            K_new = self.camera_K.copy()
            K_new[0, 0] *= self.config['fisheye_zoom']
            K_new[1, 1] *= self.config['fisheye_zoom']

            self.fisheye_map1, self.fisheye_map2 = cv2.fisheye.initUndistortRectifyMap(
                self.camera_K,
                self.camera_D,
                np.eye(3),
                K_new,
                (self.config['image_width'], self.config['image_height']),
                cv2.CV_16SC2
            )

            # Subscribe to camera (only when trajectory 9 is active)
            self.image_sub = rospy.Subscriber(
                self.config['camera_topic'],
                Image,
                self._image_callback,
                queue_size=1,
                buff_size=2**24
            )

            # Create debug image publisher
            self.debug_image_pub = rospy.Publisher(
                '/hand_tracking/debug_image',
                Image,
                queue_size=1
            )

            self.initialized = True
            self.active = True  # Enable hand tracking
            rospy.loginfo("Hand tracking initialized successfully")
            rospy.loginfo(f"Camera: {self.config['camera_topic']} ({self.config['image_width']}x{self.config['image_height']})")
            rospy.loginfo(f"Fisheye zoom: {self.config['fisheye_zoom']}x")
            rospy.loginfo("Debug image: /hand_tracking/debug_image")
            rospy.loginfo("Show your hand to the camera to begin tracking...")

            return True

        except ImportError as e:
            rospy.logerr(f"Failed to import mediapipe: {e}")
            rospy.logerr("Install with: pip install mediapipe")
            return False
        except Exception as e:
            rospy.logerr(f"Hand tracking initialization failed: {e}")
            return False

    def _image_callback(self, msg):
        """
        Process camera images for hand detection.

        Args:
            msg: ROS Image message
        """
        if not self.active or self.mediapipe_hands is None:
            return

        try:
            # Check image age to prevent processing stale images
            image_age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if image_age > 0.5:
                rospy.logwarn_throttle(5.0, f"Stale image: {image_age:.2f}s old")
                return

            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Undistort fisheye image
            if self.fisheye_map1 is not None:
                cv_image = cv2.remap(cv_image, self.fisheye_map1, self.fisheye_map2, cv2.INTER_LINEAR)

            # MediaPipe requires RGB
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Detect hands
            results = self.mediapipe_hands.process(image_rgb)

            # Process detection results
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                h, w, _ = cv_image.shape

                # Track middle finger MCP (landmark 9)
                palm_center = hand_landmarks.landmark[9]
                cx = int(np.clip(palm_center.x * w, 0, w - 1))
                cy = int(np.clip(palm_center.y * h, 0, h - 1))

                # Calculate hand orientation (wrist to MCP)
                wrist = hand_landmarks.landmark[0]
                wrist_x = wrist.x * w
                wrist_y = wrist.y * h
                dx = cx - wrist_x
                dy = cy - wrist_y
                initial_hand_offset_rad = math.radians(15.0)  # Magic number for natural hand roll
                hand_angle = math.atan2(-dy, dx) - math.pi/2 + initial_hand_offset_rad
                hand_angle = wrap_angle(hand_angle)

                # Update tracking state
                self.target_pixel_x = cx
                self.target_pixel_y = cy
                self.hand_angle = hand_angle
                self.target_found = True
                self.last_detection_time = time.time()

                # Get hand label and confidence
                hand_label = results.multi_handedness[0].classification[0].label
                confidence = results.multi_handedness[0].classification[0].score

                rospy.loginfo_throttle(1.0, f"[HAND DETECTED] {hand_label} hand at ({cx}, {cy}), angle={math.degrees(hand_angle):+.1f}°, conf={confidence:.2f}")
                sys.stdout.flush()

                # Publish debug image with hand landmarks
                if self.debug_image_pub is not None and self.debug_image_pub.get_num_connections() > 0:
                    self.publish_debug_image(cv_image, results, (cx, cy))
            else:
                self.target_found = False
                rospy.loginfo_throttle(2.0, "[NO HAND DETECTED]")
                sys.stdout.flush()

                # Publish debug image showing no detection
                if self.debug_image_pub is not None and self.debug_image_pub.get_num_connections() > 0:
                    self.publish_debug_image(cv_image, None, None)

        except CvBridgeError as e:
            rospy.logerr_throttle(5.0, f"CvBridge error: {e}")
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Image processing error: {e}")

    def publish_debug_image(self, cv_image, results, centroid):
        """
        Publish debug image with MediaPipe hand landmarks.

        Args:
            cv_image: Original OpenCV image
            results: MediaPipe results object
            centroid: (x, y) tuple of tracking point, or None
        """
        try:
            # Create debug image
            debug_img = cv_image.copy()

            # Draw crosshairs at image center
            center_x = int(self.config['center_x'])
            center_y = int(self.config['center_y'])
            cv2.line(debug_img, (center_x - 20, center_y), (center_x + 20, center_y), (255, 255, 255), 2)
            cv2.line(debug_img, (center_x, center_y - 20), (center_x, center_y + 20), (255, 255, 255), 2)

            # Draw dead zone rectangle
            dead_x = self.config['pixel_deadzone_x']
            dead_y = self.config['pixel_deadzone_y']
            cv2.rectangle(debug_img,
                         (center_x - dead_x, center_y - dead_y),
                         (center_x + dead_x, center_y + dead_y),
                         (255, 255, 0), 2)

            if results and results.multi_hand_landmarks:
                # Draw hand landmarks and connections
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        debug_img,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=3),
                        self.mp_drawing.DrawingSpec(color=(0, 255, 255), thickness=2)
                    )

                if centroid is not None:
                    # Draw tracking point (palm center)
                    cv2.circle(debug_img, centroid, 10, (0, 0, 255), -1)

                    # Draw hand orientation vector (wrist to middle finger MCP)
                    hand_landmarks = results.multi_hand_landmarks[0]
                    h, w, _ = debug_img.shape
                    wrist = hand_landmarks.landmark[0]
                    wrist_x = int(wrist.x * w)
                    wrist_y = int(wrist.y * h)
                    cv2.line(debug_img, (wrist_x, wrist_y), centroid, (255, 0, 255), 3)

                    # Draw line from center to tracking point
                    cv2.line(debug_img, (center_x, center_y), centroid, (0, 255, 255), 2)

                    # Add text with pixel offset
                    offset_x = centroid[0] - center_x
                    offset_y = centroid[1] - center_y

                    # Get hand label and confidence
                    hand_label = results.multi_handedness[0].classification[0].label
                    confidence = results.multi_handedness[0].classification[0].score

                    # Display hand info including orientation angle
                    hand_angle_deg = math.degrees(self.hand_angle)
                    text = f"{hand_label} Hand: Offset=({offset_x:+.0f},{offset_y:+.0f})px Angle={hand_angle_deg:+.1f}deg Conf={confidence:.2f}"
                    cv2.putText(debug_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                               0.6, (0, 255, 0), 2)
            else:
                # No hand detected
                cv2.putText(debug_img, "NO HAND DETECTED", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                           0.8, (0, 0, 255), 2)

            # Publish
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = rospy.Time.now()
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            rospy.logerr_throttle(5.0, f"Debug image error: {e}")

    def pixel_to_angular_error(self, pixel_x, pixel_y):
        """
        Convert pixel offset to angular error in radians.

        Args:
            pixel_x: Pixel x coordinate
            pixel_y: Pixel y coordinate

        Returns:
            Tuple of (yaw_error, pitch_error) in radians
        """
        # Calculate pixel offset from center
        pixel_error_x = pixel_x - self.config['center_x']
        pixel_error_y = pixel_y - self.config['center_y']

        # Apply dead zone
        if abs(pixel_error_x) < self.config['pixel_deadzone_x']:
            pixel_error_x = 0.0
        if abs(pixel_error_y) < self.config['pixel_deadzone_y']:
            pixel_error_y = 0.0

        # Convert to angular error (focal lengths already scaled by zoom)
        focal_x = self.config['focal_length_px'] * self.config['fisheye_zoom']
        focal_y = self.config['focal_length_py'] * self.config['fisheye_zoom']

        yaw_error = math.atan2(pixel_error_x, focal_x)
        pitch_error = -math.atan2(pixel_error_y, focal_y)

        return yaw_error, pitch_error

    def deactivate(self):
        """
        Deactivate hand tracking (resources remain initialized for next use).
        """
        rospy.loginfo("Deactivating hand tracking...")

        # Just deactivate - don't destroy resources since they're pre-initialized
        self.active = False

        # Reset tracking state
        self.target_found = False

        rospy.loginfo("Hand tracking deactivated (resources remain ready)")

    def cleanup(self):
        """
        Fully cleanup hand tracking resources.
        """
        rospy.loginfo("Cleaning up hand tracking resources...")

        # Unsubscribe from camera
        if self.image_sub is not None:
            self.image_sub.unregister()
            self.image_sub = None

        # Shutdown debug publisher
        if self.debug_image_pub is not None:
            self.debug_image_pub.unregister()
            self.debug_image_pub = None

        # Close MediaPipe hands
        if self.mediapipe_hands is not None:
            self.mediapipe_hands.close()
            self.mediapipe_hands = None

        self.initialized = False
        self.active = False

        rospy.loginfo("Hand tracking resources cleaned up")
