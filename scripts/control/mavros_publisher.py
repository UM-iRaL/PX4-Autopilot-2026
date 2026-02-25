"""
MAVROS Full State Setpoint Publisher for Geometric Control.

This module provides the MAVROSFullStatePublisher class which publishes position, velocity,
attitude, and attitude rate setpoints via MAVROS for use with the geometric_control module.
"""

import math
import time
import numpy as np
import rospy
from threading import Thread
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped
from mavros_msgs.msg import State, AttitudeTarget, PositionTarget, RCIn
from mavros_msgs.srv import (CommandBool, CommandBoolRequest, SetMode, SetModeRequest,
                              CommandLong, CommandLongRequest)
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix

# Import utilities and configuration
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.math_utils import (ned_aircraft_to_enu_baselink, enu_baselink_to_ned_aircraft,
                              body_rates_from_quaternions, euler_rates_to_body_rates_flu)
from config.setpoint_config import SetpointConfig
from trajectory.trajectory_manager import TrajectoryManager
from trajectory_loader import TrajectoryConfigLoader, ConfigValidationError


class MAVROSFullStatePublisher:
    """Main class for publishing full state setpoints via MAVROS"""

    def __init__(self, config_path=None):
        """Initialize the setpoint publisher

        Args:
            config_path: Optional path to YAML configuration file
        """
        rospy.init_node('mavros_full_state_publisher', anonymous=True)

        # Load configuration from YAML
        try:
            self.config_loader = TrajectoryConfigLoader(config_path)
        except ConfigValidationError as e:
            rospy.logerr(f"Configuration error: {e}")
            rospy.logerr("Using default configuration")
            self.config_loader = None

        # Configuration (now loads from YAML)
        self.config = SetpointConfig(self.config_loader)

        # Current vehicle state
        self.current_state = State()

        # Current pose (for trajectory feedback)
        self.current_pose = None
        self.last_pose_time = rospy.Time(0)

        # Current velocity (for auto-land error monitoring)
        self.current_velocity = None
        self.last_velocity_time = rospy.Time(0)

        # Setup setpoint messages
        self._setup_setpoints()

        # Trajectory manager (created after setpoints for access to position_target)
        self.trajectory_manager = TrajectoryManager(publisher=self, config_loader=self.config_loader)

        # Publishers
        # Position/Velocity/Acceleration publisher (goes to trajectory_setpoint)
        self.position_raw_pub = rospy.Publisher(
            'mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=10
        )

        # Attitude/Rate publisher (goes to vehicle_attitude_setpoint and vehicle_rates_setpoint)
        self.attitude_raw_pub = rospy.Publisher(
            'mavros/setpoint_raw/attitude',
            AttitudeTarget,
            queue_size=10
        )

        # Subscribers
        self.state_sub = rospy.Subscriber(
            'mavros/state',
            State,
            self._state_callback
        )

        self.pose_sub = rospy.Subscriber(
            'mavros/local_position/pose',
            PoseStamped,
            self._pose_callback,
            queue_size=1
        )

        self.velocity_sub = rospy.Subscriber(
            'mavros/local_position/velocity_local',
            TwistStamped,
            self._velocity_callback,
            queue_size=1
        )

        # RC input subscriber for emergency landing trigger
        self._rc_emergency_channel = 4  # Channel 5 (0-indexed)
        self._rc_emergency_threshold = 1200  # Above this = emergency landing
        self._rc_last_channel_value = None  # Track previous value for edge detection
        self._rc_emergency_triggered = False  # Prevent re-triggering
        self.rc_sub = rospy.Subscriber(
            'mavros/rc/in',
            RCIn,
            self._rc_in_callback,
            queue_size=1
        )

        # Service clients
        self.arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.command_client = rospy.ServiceProxy('mavros/cmd/command', CommandLong)

        # Publishing thread
        self.publish_thread = None
        self.running = False

        rospy.loginfo("MAVROS Full State Setpoint Publisher initialized")
        rospy.loginfo(f"Target position: x={self.config.POSITION['x']}, y={self.config.POSITION['y']}, z={self.config.POSITION['z']}")
        rospy.loginfo(f"Target velocity: vx={self.config.VELOCITY['vx']}, vy={self.config.VELOCITY['vy']}, vz={self.config.VELOCITY['vz']}")
        rospy.loginfo(f"Target orientation: roll={math.degrees(self.config.ORIENTATION['roll']):.1f}deg, pitch={math.degrees(self.config.ORIENTATION['pitch']):.1f}deg, yaw={math.degrees(self.config.ORIENTATION['yaw']):.1f}deg")
        rospy.loginfo(f"Target rates: rollspeed={math.degrees(self.config.ANGULAR_RATES['rollspeed']):.1f}deg/s, pitchspeed={math.degrees(self.config.ANGULAR_RATES['pitchspeed']):.1f}deg/s, yawspeed={math.degrees(self.config.ANGULAR_RATES['yawspeed']):.1f}deg/s")

    def _setup_setpoints(self):
        """Setup the setpoint messages"""

        # Position/Velocity/Acceleration message (PositionTarget)
        self.position_target = PositionTarget()
        self.position_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask: Set bits to IGNORE specific fields
        # We enable: position (x,y,z), velocity (vx,vy,vz), acceleration (ax,ay,az), yaw, yaw_rate
        # We treat acceleration as acceleration (not force)
        self.position_target.type_mask = 0  # Enable all fields (position, velocity, acceleration)

        self.position_target.position.x = self.config.POSITION['x']
        self.position_target.position.y = self.config.POSITION['y']
        self.position_target.position.z = self.config.POSITION['z']

        self.position_target.velocity.x = self.config.VELOCITY['vx']
        self.position_target.velocity.y = self.config.VELOCITY['vy']
        self.position_target.velocity.z = self.config.VELOCITY['vz']

        self.position_target.acceleration_or_force.x = self.config.ACCELERATION['ax']
        self.position_target.acceleration_or_force.y = self.config.ACCELERATION['ay']
        self.position_target.acceleration_or_force.z = self.config.ACCELERATION['az']

        self.position_target.yaw = self.config.ORIENTATION['yaw']
        self.position_target.yaw_rate = self.config.ANGULAR_RATES['yawspeed']

        # Attitude/Rate message (AttitudeTarget)
        self.attitude_target = AttitudeTarget()

        # Type mask: Set bits to IGNORE specific fields
        # We enable: attitude (orientation quaternion) and body rates (roll/pitch/yaw rates)
        # We ignore: thrust (comes from position controller)
        self.attitude_target.type_mask = AttitudeTarget.IGNORE_THRUST

        roll  = self.config.ORIENTATION['roll']
        pitch = self.config.ORIENTATION['pitch']
        yaw = self.config.ORIENTATION['yaw']

        # Track last commanded Euler angles (for safe landing trajectory calculation)
        self.last_commanded_roll = roll
        self.last_commanded_pitch = pitch
        self.last_commanded_yaw = yaw

        # Track last commanded quaternion (NED aircraft frame) to avoid
        # gimbal lock when holding orientation at pitch = ±90°
        self.last_commanded_quat_ned = quaternion_from_euler(yaw, pitch, roll, axes='rzyx')

        # Previous quaternion for body rate computation via finite differencing
        self.prev_quat_ned = self.last_commanded_quat_ned.copy()

        # Convert from NED Euler angles (roll, pitch, yaw) to quaternion in NED aircraft frame
        # PX4 uses aerospace convention: intrinsic rotations in Z-Y-X order (yaw, pitch, roll)
        q_ned_aircraft = quaternion_from_euler(yaw, pitch, roll, axes='rzyx')  # [x, y, z, w]

        # Convert to ENU base_link frame for MAVROS (MAVROS will convert back internally)
        q_enu_baselink = ned_aircraft_to_enu_baselink(q_ned_aircraft)

        self.attitude_target.orientation = Quaternion(x=q_enu_baselink[0], y=q_enu_baselink[1],
                                                      z=q_enu_baselink[2], w=q_enu_baselink[3])

        # Set initial body rates (defaults are zero; overwritten by trajectory loop)
        self.attitude_target.body_rate.x = self.config.ANGULAR_RATES['rollspeed']
        self.attitude_target.body_rate.y = self.config.ANGULAR_RATES['pitchspeed']
        self.attitude_target.body_rate.z = self.config.ANGULAR_RATES['yawspeed']

        # Thrust (normalized 0-1, ignored in our case)
        self.attitude_target.thrust = 0.5

    def _state_callback(self, msg):
        """Callback for vehicle state updates"""
        self.current_state = msg

    def _pose_callback(self, msg):
        """Callback for pose updates - extract current attitude"""
        self.current_pose = msg
        self.last_pose_time = rospy.Time.now()

        # Store position for WaitTrajectory feedback and auto-land error monitoring.
        # Both MAVROS pose and PositionTarget setpoints are in ENU frame
        # (MAVROS converts FRAME_LOCAL_NED setpoints from ENU→NED internally,
        # so the values we write into PositionTarget are actually ENU).
        self.trajectory_manager.current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # Extract ENU orientation from MAVROS
        q = msg.pose.orientation
        q_enu = [q.x, q.y, q.z, q.w]

        # Convert ENU -> NED for control logic
        q_ned = enu_baselink_to_ned_aircraft(q_enu)

        # Convert to Euler angles
        roll, pitch, yaw = euler_from_quaternion(q_ned)

        # Update trajectory manager's current attitude
        self.trajectory_manager.current_roll = roll
        self.trajectory_manager.current_pitch = pitch
        self.trajectory_manager.current_yaw = yaw

        # Store orientation quaternion for auto-land error monitoring (in NED frame)
        self.trajectory_manager.current_orientation_quat = np.array(q_ned)

    def _velocity_callback(self, msg):
        """Callback for velocity updates - for auto-land error monitoring"""
        self.current_velocity = msg
        self.last_velocity_time = rospy.Time.now()

        # Store linear velocity in trajectory manager for error monitoring.
        # Both velocity_local and PositionTarget setpoints are in ENU frame
        # (MAVROS converts FRAME_LOCAL_NED setpoints from ENU→NED internally,
        # so the velocity values we write into PositionTarget are actually ENU).
        self.trajectory_manager.current_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])

        # velocity_local twist.angular is in the world frame (ENU).
        # Rotate to body frame (FLU) to match commanded body angular velocity.
        ang_vel_world = np.array([
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ])
        if self.current_pose is not None:
            q = self.current_pose.pose.orientation
            # quaternion_matrix returns body-to-world rotation for this ENU quaternion
            R_body_to_world = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
            # world-to-body is the transpose (R is orthogonal)
            self.trajectory_manager.current_angular_velocity = R_body_to_world.T @ ang_vel_world
        else:
            self.trajectory_manager.current_angular_velocity = ang_vel_world

    def _rc_in_callback(self, msg):
        """Callback for RC input - triggers emergency landing on channel 5 switch.

        Monitors RC channel 5 (index 4) for a falling edge from ~1494 to ~982.
        """
        # ------------------------------------------------------------------
        # Emergency landing check (channel 5, index 4)
        # ------------------------------------------------------------------
        if len(msg.channels) > self._rc_emergency_channel:
            current_value = msg.channels[self._rc_emergency_channel]

            if self._rc_last_channel_value is not None and not self._rc_emergency_triggered:
                was_high = self._rc_last_channel_value >= self._rc_emergency_threshold
                is_low = current_value < self._rc_emergency_threshold

                if was_high and is_low and self.current_state.armed:
                    self._rc_emergency_triggered = True
                    rospy.logwarn("")
                    rospy.logwarn("=" * 60)
                    rospy.logwarn("RC EMERGENCY LANDING TRIGGERED (CH5: %d -> %d)",
                                  self._rc_last_channel_value, current_value)
                    rospy.logwarn("=" * 60)
                    self.trajectory_manager.start_trajectory(12)

            self._rc_last_channel_value = current_value

    def _publish_setpoints(self):
        """Publishing thread function"""
        rate = rospy.Rate(self.config.PUBLISH_RATE_HZ)

        # Track last waypoint index to detect transitions
        last_waypoint_index = -1

        while not rospy.is_shutdown() and self.running:
            # Check if trajectory is active and update setpoints
            if self.trajectory_manager.is_active():
                setpoint = self.trajectory_manager.get_current_setpoint()

                # Check for waypoint transition and apply L1 parameters if defined
                # This works for trajectory 10 (WaitTrajectory)
                if (self.trajectory_manager.current_trajectory == 10 and
                    self.trajectory_manager.wait_trajectory_active and
                    self.trajectory_manager.wait_trajectory is not None):

                    current_waypoint_index = self.trajectory_manager.wait_trajectory.current_waypoint_index

                    # Detect waypoint transition
                    if current_waypoint_index != last_waypoint_index:
                        last_waypoint_index = current_waypoint_index

                        # Get L1 parameters for this waypoint
                        l1_params = self.trajectory_manager.wait_trajectory.get_current_waypoint_l1_params()

                        if l1_params is not None:
                            rospy.loginfo(f"Waypoint {current_waypoint_index} - Applying L1 parameters: {l1_params}")
                            # Apply parameters in order: individual channels first, then master switch
                            self.set_l1_moment_enable_param(l1_params.get('enable_moment', 0))
                            self.set_l1_force_enable_param(l1_params.get('enable_force', 0))
                            self.set_l1_active_param(l1_params.get('enable', 0))

                # Check if takeoff trajectory (6) is complete and enable L1 force channel
                if (self.trajectory_manager.current_trajectory == 6 and
                    not self.trajectory_manager.takeoff_complete):
                    elapsed = time.time() - self.trajectory_manager.trajectory_start_time
                    if elapsed >= self.config.TAKEOFF['duration']:
                        rospy.loginfo("Takeoff trajectory complete - enabling L1 force channel")
                        self.set_l1_force_enable_param(1)  # Enable force channel now that takeoff is complete
                        self.trajectory_manager.takeoff_complete = True

                if setpoint is not None:
                    # Ensure position commands are enabled when trajectory is active
                    self.position_target.type_mask = 0  # Enable all fields

                    # Update position
                    self.position_target.position.x = setpoint['position']['x']
                    self.position_target.position.y = setpoint['position']['y']
                    self.position_target.position.z = setpoint['position']['z']

                    # Update velocity
                    self.position_target.velocity.x = setpoint['velocity']['vx']
                    self.position_target.velocity.y = setpoint['velocity']['vy']
                    self.position_target.velocity.z = setpoint['velocity']['vz']

                    # Update acceleration if provided by trajectory
                    if 'acceleration' in setpoint:
                        self.position_target.acceleration_or_force.x = setpoint['acceleration']['ax']
                        self.position_target.acceleration_or_force.y = setpoint['acceleration']['ay']
                        self.position_target.acceleration_or_force.z = setpoint['acceleration']['az']

                    # Update orientation from trajectory setpoint
                    # Check if trajectory provides quaternion directly or Euler angles
                    if 'orientation_quat' in setpoint:
                        # Direct quaternion provided (already in NED aircraft frame)
                        quat = setpoint['orientation_quat']
                        q_ned_aircraft = np.array([quat['x'], quat['y'], quat['z'], quat['w']])

                        # Convert to ENU base_link frame for MAVROS
                        q_enu_baselink = ned_aircraft_to_enu_baselink(q_ned_aircraft)
                        self.attitude_target.orientation = Quaternion(x=q_enu_baselink[0], y=q_enu_baselink[1],
                                                                      z=q_enu_baselink[2], w=q_enu_baselink[3])

                        # Store quaternion directly for hold functionality (avoids gimbal lock)
                        self.last_commanded_quat_ned = q_ned_aircraft.copy()

                        # Extract Euler angles for yaw field and landing trajectory.
                        # These may be discontinuous at pitch=±90° but are only used
                        # for the yaw field in PositionTarget and as a fallback.
                        yaw, pitch, roll = euler_from_quaternion(q_ned_aircraft, axes='rzyx')
                        self.last_commanded_roll = roll
                        self.last_commanded_pitch = pitch
                        self.last_commanded_yaw = yaw
                        self.position_target.yaw = yaw

                    else:
                        # Euler angles provided
                        roll  = setpoint['orientation']['roll']
                        pitch = setpoint['orientation']['pitch']
                        yaw = setpoint['orientation']['yaw']

                        # Track last commanded Euler angles
                        self.last_commanded_roll = roll
                        self.last_commanded_pitch = pitch
                        self.last_commanded_yaw = yaw

                        # Convert from NED Euler angles to quaternion in NED aircraft frame
                        q_ned_aircraft = quaternion_from_euler(yaw, pitch, roll, axes='rzyx')
                        # Store quaternion for hold functionality
                        self.last_commanded_quat_ned = q_ned_aircraft.copy()
                        # Convert to ENU base_link frame for MAVROS
                        q_enu_baselink = ned_aircraft_to_enu_baselink(q_ned_aircraft)
                        self.attitude_target.orientation = Quaternion(x=q_enu_baselink[0], y=q_enu_baselink[1],
                                                                      z=q_enu_baselink[2], w=q_enu_baselink[3])
                        self.position_target.yaw = yaw

                    # Convert trajectory Euler rates to body rates (FLU).
                    #
                    # Trajectories output d(euler)/dt (rollspeed, pitchspeed, yawspeed).
                    # PX4 AttitudeTarget.body_rate expects body angular velocity.
                    # The ZYX kinematic matrix (euler_rates_to_body_rates_flu) is
                    # well-conditioned in the forward direction at ALL pitch angles —
                    # the singularity only exists in the inverse (body → Euler).
                    #
                    # When use_explicit_rates=True (e.g. emergency landing rate damping),
                    # rates are already in body frame — use them directly.
                    if setpoint.get('use_explicit_rates', False):
                        body_rates_flu = np.array([
                            setpoint['rates']['rollspeed'],
                            setpoint['rates']['pitchspeed'],
                            setpoint['rates']['yawspeed']
                        ])
                    else:
                        body_rates_flu = euler_rates_to_body_rates_flu(
                            roll, pitch,
                            setpoint['rates']['rollspeed'],
                            setpoint['rates']['pitchspeed'],
                            setpoint['rates']['yawspeed']
                        )
                    self.prev_quat_ned = q_ned_aircraft.copy()
                    self.attitude_target.body_rate.x = body_rates_flu[0]
                    self.attitude_target.body_rate.y = body_rates_flu[1]
                    self.attitude_target.body_rate.z = body_rates_flu[2]
                    self.position_target.yaw_rate = setpoint['rates']['yawspeed']

                    # Update commanded state for auto-land error monitoring
                    # Use the NED aircraft frame quaternion for orientation comparison
                    cmd_pos = np.array([
                        setpoint['position']['x'],
                        setpoint['position']['y'],
                        setpoint['position']['z']
                    ])
                    cmd_vel = np.array([
                        setpoint['velocity']['vx'],
                        setpoint['velocity']['vy'],
                        setpoint['velocity']['vz']
                    ])
                    # Reuse the quaternion-based body rates computed above for
                    # auto-land error monitoring (avoids gimbal lock spikes).
                    cmd_ang_vel = body_rates_flu
                    self.trajectory_manager.update_commanded_state(
                        position=cmd_pos,
                        velocity=cmd_vel,
                        orientation_quat=q_ned_aircraft,
                        angular_velocity=cmd_ang_vel
                    )

                    # Check for excessive trajectory error (auto-land safety)
                    if self.trajectory_manager.check_and_trigger_auto_land():
                        rospy.logwarn("Auto-land triggered - continuing with landing trajectory")
                        # Get new setpoint from landing trajectory
                        setpoint = self.trajectory_manager.get_current_setpoint()
                        if setpoint is not None:
                            # Update ALL fields from new trajectory setpoint
                            self.position_target.position.x = setpoint['position']['x']
                            self.position_target.position.y = setpoint['position']['y']
                            self.position_target.position.z = setpoint['position']['z']
                            self.position_target.velocity.x = setpoint['velocity']['vx']
                            self.position_target.velocity.y = setpoint['velocity']['vy']
                            self.position_target.velocity.z = setpoint['velocity']['vz']

                            # Update orientation from new setpoint
                            if 'orientation' in setpoint:
                                roll = setpoint['orientation']['roll']
                                pitch = setpoint['orientation']['pitch']
                                yaw = setpoint['orientation']['yaw']
                                q_ned_aircraft = quaternion_from_euler(yaw, pitch, roll, axes='rzyx')
                                self.last_commanded_quat_ned = q_ned_aircraft.copy()
                                q_enu_baselink = ned_aircraft_to_enu_baselink(q_ned_aircraft)
                                self.attitude_target.orientation = Quaternion(
                                    x=q_enu_baselink[0], y=q_enu_baselink[1],
                                    z=q_enu_baselink[2], w=q_enu_baselink[3])
                                self.position_target.yaw = yaw

                            # Convert Euler rates to body rates for new setpoint
                            if setpoint.get('use_explicit_rates', False):
                                body_rates_flu = np.array([
                                    setpoint['rates']['rollspeed'],
                                    setpoint['rates']['pitchspeed'],
                                    setpoint['rates']['yawspeed']
                                ])
                            else:
                                body_rates_flu = euler_rates_to_body_rates_flu(
                                    roll, pitch,
                                    setpoint['rates']['rollspeed'],
                                    setpoint['rates']['pitchspeed'],
                                    setpoint['rates']['yawspeed']
                                )
                            self.attitude_target.body_rate.x = body_rates_flu[0]
                            self.attitude_target.body_rate.y = body_rates_flu[1]
                            self.attitude_target.body_rate.z = body_rates_flu[2]
                            self.position_target.yaw_rate = setpoint['rates']['yawspeed']
                            self.prev_quat_ned = self.last_commanded_quat_ned.copy()

                    # Check if landing is complete
                    if 'landing_complete' in setpoint and setpoint['landing_complete']:
                        rospy.loginfo("Landing complete - disarming vehicle")
                        self.disarm_vehicle(force=False)  # Normal disarm
                        self.trajectory_manager.stop_trajectory()

            else:
                # No active trajectory
                # Before takeoff (when still near ground), don't send position commands
                # After takeoff has occurred at least once, hold last commanded pose to freeze in place
                if self.trajectory_manager.takeoff_complete and self.current_pose is not None:
                    # Post-takeoff: hold last commanded pose (freeze in place)
                    # Note: position_target already contains last commanded position, no need to update
                    if self.current_state.armed:
                        rospy.loginfo_throttle(5.0, "No active trajectory - holding last commanded pose")

                    # Velocity and acceleration: zero
                    self.position_target.velocity.x = 0.0
                    self.position_target.velocity.y = 0.0
                    self.position_target.velocity.z = 0.0
                    self.position_target.acceleration_or_force.x = 0.0
                    self.position_target.acceleration_or_force.y = 0.0
                    self.position_target.acceleration_or_force.z = 0.0

                    # Attitude: hold last commanded attitude using stored quaternion
                    # (avoids gimbal lock from Euler->quaternion round trip at pitch=±90°)
                    q_enu_baselink = ned_aircraft_to_enu_baselink(self.last_commanded_quat_ned)
                    self.attitude_target.orientation = Quaternion(x=q_enu_baselink[0], y=q_enu_baselink[1],
                                                                  z=q_enu_baselink[2], w=q_enu_baselink[3])
                    self.position_target.yaw = self.last_commanded_yaw

                    # Rates: zero
                    self.attitude_target.body_rate.x = 0.0
                    self.attitude_target.body_rate.y = 0.0
                    self.attitude_target.body_rate.z = 0.0
                    self.position_target.yaw_rate = 0.0
                else:
                    # Pre-takeoff: explicitly set initial setpoints from config
                    # This maintains position=(0,0,0) with az=-5.0 to keep drone on ground until takeoff trajectory starts
                    if self.current_state.armed:
                        rospy.loginfo_throttle(5.0, "No active trajectory - ready for takeoff (press '6')")

                    # Explicitly set position, velocity, acceleration from config each iteration
                    # to ensure they're not accidentally overwritten by other code paths
                    self.position_target.position.x = self.config.POSITION['x']
                    self.position_target.position.y = self.config.POSITION['y']
                    self.position_target.position.z = self.config.POSITION['z']

                    self.position_target.velocity.x = self.config.VELOCITY['vx']
                    self.position_target.velocity.y = self.config.VELOCITY['vy']
                    self.position_target.velocity.z = self.config.VELOCITY['vz']

                    self.position_target.acceleration_or_force.x = self.config.ACCELERATION['ax']
                    self.position_target.acceleration_or_force.y = self.config.ACCELERATION['ay']
                    self.position_target.acceleration_or_force.z = self.config.ACCELERATION['az']

                    self.position_target.yaw = self.config.ORIENTATION['yaw']
                    self.position_target.yaw_rate = self.config.ANGULAR_RATES['yawspeed']

            # Update timestamps
            current_time = rospy.Time.now()
            self.position_target.header.stamp = current_time
            self.attitude_target.header.stamp = current_time

            # Publish setpoints
            self.position_raw_pub.publish(self.position_target)
            self.attitude_raw_pub.publish(self.attitude_target)

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def start_publishing(self):
        """Start publishing setpoints in a separate thread"""
        if not self.running:
            self.running = True
            self.publish_thread = Thread(target=self._publish_setpoints)
            self.publish_thread.daemon = True
            self.publish_thread.start()
            rospy.loginfo("Started publishing full state setpoints")

    def stop_publishing(self):
        """Stop publishing setpoints"""
        self.running = False
        if self.publish_thread:
            self.publish_thread.join()
        rospy.loginfo("Stopped publishing setpoints")

    def wait_for_connection(self):
        """Wait for MAVROS connection"""
        rospy.loginfo("Waiting for MAVROS connection...")

        # Wait for state messages
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.1)

        rospy.loginfo("MAVROS connected")

    def set_offboard_mode(self):
        """Set vehicle to OFFBOARD mode"""
        rospy.loginfo("Setting OFFBOARD mode...")

        set_mode_req = SetModeRequest()
        set_mode_req.custom_mode = "OFFBOARD"

        # Try to set mode
        for i in range(self.config.TIMEOUT_SEC * 2):  # 2Hz retry rate
            if self.current_state.mode == "OFFBOARD":
                rospy.loginfo("OFFBOARD mode set successfully")
                return True

            try:
                response = self.set_mode_client.call(set_mode_req)
                if response.mode_sent:
                    rospy.loginfo("OFFBOARD mode request sent")
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: {}".format(e))

            rospy.sleep(0.5)

        rospy.logerr("Failed to set OFFBOARD mode")
        return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        # Block arming if RC emergency switch is already in the triggered position (low)
        if (self._rc_last_channel_value is not None and
                self._rc_last_channel_value < self._rc_emergency_threshold):
            rospy.logerr("ARMING BLOCKED: RC emergency switch (CH5) is already ON (%d)",
                         self._rc_last_channel_value)
            rospy.logerr("Flip the emergency switch OFF before arming")
            return False

        rospy.loginfo("Arming vehicle...")

        arm_req = CommandBoolRequest()
        arm_req.value = True

        # Send arming command once
        command_sent = False
        try:
            response = self.arming_client.call(arm_req)
            if response.success:
                rospy.loginfo("Arming command sent successfully")
                command_sent = True
            else:
                rospy.logwarn("Arming command rejected")
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return False

        # Wait for vehicle to be armed (check state updates)
        if command_sent:
            for _ in range(self.config.TIMEOUT_SEC * 2):  # 2Hz retry rate
                if self.current_state.armed:
                    rospy.loginfo("Vehicle armed successfully")
                    # Disable L1 on arming to ensure clean state (will be enabled when takeoff trajectory starts)
                    rospy.loginfo("Disabling L1 adaptive controller (will enable when takeoff trajectory starts)...")
                    self.set_l1_active_param(0)
                    self.set_l1_force_enable_param(0)
                    self.set_l1_moment_enable_param(0)
                    return True
                rospy.sleep(0.5)

        rospy.logerr("Failed to arm vehicle")
        return False

    def disarm_vehicle(self, force=False):
        """
        Disarm the vehicle

        Args:
            force: If True, force disarm even if vehicle is in air or other safety checks fail

        Returns:
            bool: True if disarm was successful
        """
        if force:
            rospy.logwarn("Force disarming vehicle...")
        else:
            rospy.loginfo("Disarming vehicle...")

        try:
            if force:
                # Use MAVLink command for force disarm
                cmd_req = CommandLongRequest()
                cmd_req.broadcast = False
                cmd_req.command = 400  # MAV_CMD_COMPONENT_ARM_DISARM
                cmd_req.confirmation = 0
                cmd_req.param1 = 0.0  # Disarm
                cmd_req.param2 = 21196.0  # Force disarm magic number
                cmd_req.param3 = 0.0
                cmd_req.param4 = 0.0
                cmd_req.param5 = 0.0
                cmd_req.param6 = 0.0
                cmd_req.param7 = 0.0

                response = self.command_client.call(cmd_req)
                if response.success:
                    rospy.loginfo("Vehicle force disarmed")
                    # Disable L1 adaptive control when disarmed
                    self.set_l1_active_param(0)
                    self.set_l1_force_enable_param(0)
                    self.set_l1_moment_enable_param(0)
                    return True
                else:
                    rospy.logwarn("Force disarm failed: result={}".format(response.result))
            else:
                # Normal disarm using arming service
                arm_req = CommandBoolRequest()
                arm_req.value = False

                response = self.arming_client.call(arm_req)
                if response.success:
                    rospy.loginfo("Vehicle disarmed")
                    # Disable L1 adaptive control when disarmed
                    self.set_l1_active_param(0)
                    self.set_l1_force_enable_param(0)
                    self.set_l1_moment_enable_param(0)
                    return True

        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))

        return False

    def terminate_flight(self):
        """
        Send flight termination command

        This triggers the flight termination system which will immediately
        stop all motors. Use with extreme caution - only for emergencies.

        Returns:
            bool: True if command was sent successfully
        """
        if not self.current_state.armed:
            rospy.logwarn("Vehicle is not armed - flight termination not needed")
            return False

        rospy.logwarn("SENDING FLIGHT TERMINATION COMMAND!")
        rospy.logwarn("Motors will stop immediately!")

        try:
            cmd_req = CommandLongRequest()
            cmd_req.broadcast = False
            cmd_req.command = 185  # MAV_CMD_DO_FLIGHTTERMINATION
            cmd_req.confirmation = 0
            cmd_req.param1 = 1.0  # Enable termination
            cmd_req.param2 = 0.0
            cmd_req.param3 = 0.0
            cmd_req.param4 = 0.0
            cmd_req.param5 = 0.0
            cmd_req.param6 = 0.0
            cmd_req.param7 = 0.0

            response = self.command_client.call(cmd_req)
            if response.success:
                rospy.logwarn("Flight termination command sent successfully")
                return True
            else:
                rospy.logerr("Flight termination command failed: result={}".format(response.result))
                return False

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return False

    def set_rcl_except_param(self):
        """Set COM_RCL_EXCEPT parameter to allow offboard mode"""
        rospy.loginfo("Setting COM_RCL_EXCEPT parameter...")

        try:
            from mavros_msgs.srv import ParamSet, ParamSetRequest
            param_set_client = rospy.ServiceProxy('mavros/param/set', ParamSet)
            rospy.wait_for_service('mavros/param/set', timeout=5.0)

            param_req = ParamSetRequest()
            param_req.param_id = "COM_RCL_EXCEPT"
            param_req.value.integer = 4  # Bit 2 set

            response = param_set_client.call(param_req)
            if response.success:
                rospy.loginfo("COM_RCL_EXCEPT parameter set successfully")
                return True
            else:
                rospy.logwarn("Failed to set COM_RCL_EXCEPT parameter")

        except Exception as e:
            rospy.logwarn("Could not set parameter: {}".format(e))

        return False

    def set_l1_active_param(self, value):
        """
        Set L1AD_ACTIVE parameter to enable/disable L1 adaptive control master switch

        Args:
            value: 1 to enable, 0 to disable L1 adaptive control master switch

        Returns:
            bool: True if parameter was set successfully
        """
        param_name = "L1AD_ACTIVE"
        rospy.loginfo("Setting {} parameter to {}...".format(param_name, value))

        try:
            from mavros_msgs.srv import ParamSet, ParamSetRequest
            param_set_client = rospy.ServiceProxy('mavros/param/set', ParamSet)
            rospy.wait_for_service('mavros/param/set', timeout=5.0)

            param_req = ParamSetRequest()
            param_req.param_id = param_name
            param_req.value.integer = value

            response = param_set_client.call(param_req)
            if response.success:
                rospy.loginfo("{} parameter set to {} successfully".format(param_name, value))
                return True
            else:
                rospy.logwarn("Failed to set {} parameter to {}".format(param_name, value))

        except Exception as e:
            rospy.logwarn("Could not set {} parameter: {}".format(param_name, e))

        return False

    def set_l1_force_enable_param(self, value):
        """
        Set L1AD_FORCE_EN parameter to enable/disable L1 adaptive force channel

        Args:
            value: 1 to enable, 0 to disable L1 adaptive force channel

        Returns:
            bool: True if parameter was set successfully
        """
        param_name = "L1AD_FORCE_EN"
        rospy.loginfo("Setting {} parameter to {}...".format(param_name, value))

        try:
            from mavros_msgs.srv import ParamSet, ParamSetRequest
            param_set_client = rospy.ServiceProxy('mavros/param/set', ParamSet)
            rospy.wait_for_service('mavros/param/set', timeout=5.0)

            param_req = ParamSetRequest()
            param_req.param_id = param_name
            param_req.value.integer = value

            response = param_set_client.call(param_req)
            if response.success:
                rospy.loginfo("{} parameter set to {} successfully".format(param_name, value))
                return True
            else:
                rospy.logwarn("Failed to set {} parameter to {}".format(param_name, value))

        except Exception as e:
            rospy.logwarn("Could not set {} parameter: {}".format(param_name, e))

        return False

    def set_l1_moment_enable_param(self, value):
        """
        Set L1AD_MOMENT_EN parameter to enable/disable L1 adaptive moment channel

        Args:
            value: 1 to enable, 0 to disable L1 adaptive moment channel

        Returns:
            bool: True if parameter was set successfully
        """
        param_name = "L1AD_MOMENT_EN"
        rospy.loginfo("Setting {} parameter to {}...".format(param_name, value))

        try:
            from mavros_msgs.srv import ParamSet, ParamSetRequest
            param_set_client = rospy.ServiceProxy('mavros/param/set', ParamSet)
            rospy.wait_for_service('mavros/param/set', timeout=5.0)

            param_req = ParamSetRequest()
            param_req.param_id = param_name
            param_req.value.integer = value

            response = param_set_client.call(param_req)
            if response.success:
                rospy.loginfo("{} parameter set to {} successfully".format(param_name, value))
                return True
            else:
                rospy.logwarn("Failed to set {} parameter to {}".format(param_name, value))

        except Exception as e:
            rospy.logwarn("Could not set {} parameter: {}".format(param_name, e))

        return False
