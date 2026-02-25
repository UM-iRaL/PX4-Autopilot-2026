#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
MAVROS Full State Setpoint Publisher for Geometric Control

This script publishes position, velocity, attitude, and attitude rate setpoints
via MAVROS for use with the geometric_control module. It publishes to multiple
MAVROS topics that get mapped to standard PX4 UORB messages.

Usage:
    python mavros_full_state_setpoint_publisher.py

The script includes 10 predefined trajectories to showcase drone capabilities:
    1 - Roll Sweep: Demonstrates full roll range from -π to π
    2 - Pitch Sweep: Demonstrates full pitch range from -π to π
    3 - Yaw Sweep: Demonstrates full yaw range from -π to π
    4 - Combined Rotation: Simultaneous rotation on all three axes
    5 - Circular Path: Circular motion with orientation changes
    6 - Takeoff: Smooth takeoff with acceleration ramp (configurable)
    7 - Landing: Smooth controlled descent to ground (configurable)
    8 - Pitch Two-Step: Two-step pitch sweep (0° → max → 0°)
    9 - Hand Tracking: Vision-based attitude control using camera and MediaPipe
    10 - WaitTrajectory: Time-scaled path following

Trajectory Controls (when drone is armed):
    Press '1-9' - Select and start trajectory
    Press 'w' - Start WaitTrajectory (trajectory 10)
    Press '0' - Stop current trajectory

Emergency Controls (when drone is armed):
    Press 'L' - Emergency landing (reactive stabilization + descent)
    Press 'T' - Send flight termination command (IMMEDIATE motor stop)
    Press 'D' - Force disarm (bypasses safety checks)
    Press 'Q' - Normal disarm and quit
    Press Ctrl+C - Exit and force disarm
"""

import signal

import rospy
from mavros_msgs.srv import SetModeRequest

# Import modularized components
from control.mavros_publisher import MAVROSFullStatePublisher
from utils.keyboard_handler import KeyboardHandler, KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT


# Global flag for signal handling
_shutdown_requested = False


def _signal_handler(signum, frame):
    """Handle Ctrl+C signal immediately."""
    global _shutdown_requested
    _shutdown_requested = True


def main():
    """Main function"""
    import argparse

    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description='MAVROS Full State Setpoint Publisher',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default configuration
  python mavros_full_state_setpoint_publisher.py

  # Use custom configuration
  python mavros_full_state_setpoint_publisher.py --config my_config.yaml
        """
    )
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='Path to trajectory configuration YAML file (default: config/trajectory_config.yaml)'
    )

    # Parse args (filter out ROS args)
    args, unknown = parser.parse_known_args()

    # Install signal handler for immediate Ctrl+C response
    signal.signal(signal.SIGINT, _signal_handler)

    publisher = None
    try:
        # Create publisher with config path
        publisher = MAVROSFullStatePublisher(config_path=args.config)

        # Wait for MAVROS connection
        publisher.wait_for_connection()

        # Set parameter for offboard mode
        publisher.set_rcl_except_param()

        # Start publishing setpoints (required before switching to offboard)
        publisher.start_publishing()

        # Give some time for setpoints to be published
        rospy.sleep(2.0)

        # Set offboard mode
        if not publisher.set_offboard_mode():
            rospy.logerr("Failed to set offboard mode, exiting...")
            # CRITICAL: Stop publishing before exiting
            publisher.stop_publishing()
            return

        rospy.loginfo("Vehicle is now in OFFBOARD mode!")
        rospy.loginfo("")
        rospy.loginfo("=" * 50)
        rospy.loginfo("Type 'arm' and press Enter to arm the vehicle")
        rospy.loginfo("=" * 50)
        rospy.loginfo("")

        # Wait for user to type 'arm' using non-blocking input in raw mode
        import sys
        import select
        import termios
        import tty

        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set raw mode FIRST, before any user input can happen
            tty.setraw(sys.stdin.fileno())

            # Flush any buffered input
            termios.tcflush(sys.stdin, termios.TCIFLUSH)

            # Print prompt (need \r\n in raw mode for proper newlines)
            sys.stdout.write(">>> ")
            sys.stdout.flush()

            user_input = ""
            while not rospy.is_shutdown() and not _shutdown_requested:
                # Check for input with short timeout to allow interrupt checking
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    if char == '\r' or char == '\n':  # Enter key
                        sys.stdout.write('\r\n')
                        sys.stdout.flush()
                        cmd = user_input.strip().lower()
                        if cmd == "arm":
                            # Restore terminal and break
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                            break
                        elif cmd == "quit" or cmd == "q":
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                            rospy.loginfo("Quit requested - exiting...")
                            publisher.stop_publishing()
                            return
                        else:
                            # Restore briefly for rospy output
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                            rospy.loginfo("Type 'arm' to arm the vehicle, or 'quit' to exit")
                            # Back to raw mode
                            tty.setraw(sys.stdin.fileno())
                            sys.stdout.write(">>> ")
                            sys.stdout.flush()
                            user_input = ""
                    elif char == '\x03':  # Ctrl+C
                        sys.stdout.write('\r\n')
                        sys.stdout.flush()
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        rospy.loginfo("Interrupted - exiting...")
                        publisher.stop_publishing()
                        return
                    elif char == '\x7f' or char == '\x08':  # Backspace
                        if user_input:
                            user_input = user_input[:-1]
                            sys.stdout.write('\b \b')
                            sys.stdout.flush()
                    else:
                        user_input += char
                        sys.stdout.write(char)
                        sys.stdout.flush()
        finally:
            # Always restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Check if we exited due to shutdown
        if _shutdown_requested or rospy.is_shutdown():
            print()
            rospy.loginfo("Shutdown requested - exiting...")
            publisher.stop_publishing()
            return

        # Arm the vehicle
        if not publisher.arm_vehicle():
            rospy.logerr("Failed to arm vehicle, exiting...")
            # CRITICAL: Stop publishing and exit OFFBOARD mode before returning
            publisher.stop_publishing()
            # Try to exit OFFBOARD mode for safety
            try:
                set_mode_req = SetModeRequest()
                set_mode_req.custom_mode = "MANUAL"  # or "STABILIZED" depending on your setup
                publisher.set_mode_client.call(set_mode_req)
                rospy.loginfo("Exited OFFBOARD mode for safety")
            except Exception as e:
                rospy.logwarn("Could not exit OFFBOARD mode: {}".format(e))
            return

        rospy.loginfo("Vehicle is now armed! Full state setpoint control active.")
        rospy.loginfo("")
        rospy.loginfo("=" * 50)
        rospy.loginfo("TRAJECTORY CONTROLS:")
        rospy.loginfo("  Press '1' - Roll Sweep (-π to π)")
        rospy.loginfo("  Press '2' - Pitch Sweep (-π to π)")
        rospy.loginfo("  Press '3' - Yaw Sweep (-π to π)")
        rospy.loginfo("  Press '4' - Combined 3-Axis Rotation")
        rospy.loginfo("  Press '5' - Circular Path with Rotation")
        rospy.loginfo("  Press '6' - Takeoff with Acceleration Ramp (0m to {:.1f}m)".format(
            abs(publisher.config.TRAJECTORY_ANGLES['z'])))
        rospy.loginfo("  Press '7' - Landing (Duration: {:.1f}s)".format(
            publisher.config.LANDING['duration']))
        rospy.loginfo("  Press '8' - Pitch Two-Step ({:.0f}° → {:.0f}° → {:.0f}°)".format(
            publisher.config.TRAJECTORY_ANGLES['pitch_min'],
            publisher.config.TRAJECTORY_ANGLES['pitch_max'],
            publisher.config.TRAJECTORY_ANGLES['pitch_min']))
        rospy.loginfo("  Press '9' - Hand Tracking (Vision-Based Attitude Control)")
        rospy.loginfo("  Press 'w' - WaitTrajectory (Time-Scaled Path Following)")
        rospy.loginfo("  Press 'v' - Vertical Circle in YZ Plane with 360° Pitch Rotation")
        rospy.loginfo("  Press 'k' - Keyboard Teleop (Arrow=XY, ,/.=Z, a/d=Yaw)")
        rospy.loginfo("  Press '0' - Stop current trajectory (return to hold)")
        rospy.loginfo("")
        rospy.loginfo("KEYBOARD TELEOP CONTROLS (active when trajectory 13 is running):")
        rospy.loginfo("  Arrow Up/Down    - Move +Y/-Y (forward/back)")
        rospy.loginfo("  Arrow Right/Left - Move +X/-X (right/left)")
        rospy.loginfo("  '.' / ','        - Move +Z/-Z (up/down)")
        rospy.loginfo("  'i' / 'k'        - Pitch nose up/down")
        rospy.loginfo("  'j' / 'l'        - Yaw CCW/CW")
        rospy.loginfo("  'u' / 'o'        - Roll CCW/CW")
        rospy.loginfo("  Space            - Level attitude (reset roll/pitch/yaw)")
        rospy.loginfo("  'p'              - Toggle PUSH mode (4 m/s² in world -Y, disables L1 force)")
        rospy.loginfo("")
        rospy.loginfo("EMERGENCY CONTROLS:")
        rospy.loginfo("  Press 'L' - Emergency landing (reactive stabilization)")
        rospy.loginfo("  Press 'T' - Flight termination (IMMEDIATE motor stop)")
        rospy.loginfo("  Press 'F' - Force disarm (bypasses safety checks)")
        rospy.loginfo("  Press 'Q' - Normal disarm and quit")
        rospy.loginfo("=" * 50)
        rospy.loginfo("")

        # Keys that are consumed by keyboard teleop (trajectory 13) instead of
        # the main command loop when that trajectory is active.
        _KB_TELEOP_KEYS = {KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, '.', ',',
                           'i', 'k', 'j', 'l', 'u', 'o', ' ', 'p'}
        # Rotation keys stop immediately when released; position keys use a hold
        # timeout to bridge key-repeat gaps and make single taps move a visible amount.
        _KB_ROTATION_KEYS = {'i', 'k', 'j', 'l', 'u', 'o'}

        # Keep running with keyboard monitoring
        # Keyboard teleop key-hold state: track last active key and a staleness counter.
        # Terminal raw mode has no key-up events; key repeat sends repeated bytes while
        # held.  Position keys: if no new key arrives for _KB_HOLD_TIMEOUT ticks the key
        # is released.  Rotation keys: cleared immediately on the first missing tick.
        _KB_HOLD_TIMEOUT = 6  # ticks at 20 Hz ≈ 300 ms (bridges key repeat gaps + makes single taps move ~9cm)
        _kb_last_teleop_key = None
        _kb_stale_ticks = 0

        with KeyboardHandler() as kbd:
            rate = rospy.Rate(20)  # 20 Hz for responsive keyboard input
            while not rospy.is_shutdown():
                key = kbd.get_key(timeout=0.03)

                # Forward movement keys to keyboard teleop trajectory when active
                kb_teleop = publisher.trajectory_manager.trajectories.get(13)
                if publisher.trajectory_manager.current_trajectory == 13 and kb_teleop is not None:
                    if key == 'p':
                        # Toggle push mode — one-shot, not a hold key
                        push_now = kb_teleop.toggle_push()
                        if push_now:
                            publisher.set_l1_force_enable_param(0)
                        else:
                            publisher.set_l1_force_enable_param(1)
                    elif key in _KB_TELEOP_KEYS:
                        # Fresh teleop key — update hold state and reset staleness
                        _kb_last_teleop_key = key
                        _kb_stale_ticks = 0
                    elif key is not None:
                        # A non-teleop key was pressed — immediately clear the
                        # held teleop key so the old command doesn't linger.
                        _kb_last_teleop_key = None
                        _kb_stale_ticks = 0
                    else:
                        # No key this tick — rotation keys stop immediately;
                        # position keys use the hold timeout to bridge key-repeat gaps.
                        if _kb_last_teleop_key in _KB_ROTATION_KEYS:
                            _kb_last_teleop_key = None
                            _kb_stale_ticks = 0
                        else:
                            _kb_stale_ticks += 1
                            if _kb_stale_ticks >= _KB_HOLD_TIMEOUT:
                                _kb_last_teleop_key = None
                    kb_teleop.update_key(_kb_last_teleop_key)
                else:
                    # Reset hold state when teleop is not active
                    _kb_last_teleop_key = None
                    _kb_stale_ticks = 0

                if key is not None:
                    # Skip teleop motion keys — already handled above
                    if (publisher.trajectory_manager.current_trajectory == 13
                            and key in _KB_TELEOP_KEYS):
                        rate.sleep()
                        continue

                    # Trajectory selection (1-9, w=10, v=11, k=13)
                    if key in ['1', '2', '3', '4', '5', '6', '7', '8', '9', 'w', 'v', 'k']:
                        if key == 'w':
                            trajectory_id = 10
                        elif key == 'v':
                            trajectory_id = 11
                        elif key == 'k':
                            trajectory_id = 13
                        else:
                            trajectory_id = int(key)
                        if publisher.current_state.armed:
                            rospy.loginfo("")
                            rospy.loginfo("=" * 50)
                            rospy.loginfo("STARTING TRAJECTORY {}: {}".format(
                                trajectory_id,
                                publisher.trajectory_manager.get_trajectory_name(trajectory_id)))
                            rospy.loginfo("=" * 50)
                            publisher.trajectory_manager.start_trajectory(trajectory_id)
                        else:
                            rospy.logwarn("Vehicle is not armed - arm first to run trajectories")
                    # Stop trajectory
                    elif key == '0':
                        if publisher.trajectory_manager.current_trajectory == 13 and kb_teleop:
                            kb_teleop.update_key(None)
                            if kb_teleop.push_active:
                                kb_teleop.toggle_push()
                                publisher.set_l1_force_enable_param(1)
                        if publisher.trajectory_manager.is_active():
                            rospy.loginfo("Stopping current trajectory")
                            publisher.trajectory_manager.stop_trajectory()
                        else:
                            rospy.loginfo("No active trajectory to stop")
                    # Emergency controls
                    elif key.lower() == 't':
                        if publisher.current_state.armed:
                            rospy.logwarn("")
                            rospy.logwarn("TERMINATION KEY PRESSED!")
                            publisher.terminate_flight()
                            rospy.sleep(1.0)
                            break
                        else:
                            rospy.loginfo("Vehicle is not armed - ignoring termination key")
                    elif key.lower() == 'f':
                        if publisher.current_state.armed:
                            rospy.logwarn("")
                            rospy.logwarn("FORCE DISARM KEY PRESSED!")
                            publisher.disarm_vehicle(force=True)
                            rospy.sleep(0.5)
                            break
                        else:
                            rospy.loginfo("Vehicle is not armed - ignoring force disarm key")
                    elif key.lower() == 'q':
                        rospy.loginfo("Quit key pressed - exiting...")
                        break
                    elif key.lower() == 'l':
                        if publisher.trajectory_manager.current_trajectory != 13:
                            if publisher.current_state.armed:
                                rospy.logwarn("")
                                rospy.logwarn("=" * 50)
                                rospy.logwarn("EMERGENCY LANDING KEY PRESSED!")
                                rospy.logwarn("=" * 50)
                                publisher.trajectory_manager.start_trajectory(12)
                            else:
                                rospy.loginfo("Vehicle is not armed - ignoring emergency landing key")
                    elif key == '\x03':  # Ctrl+C
                        rospy.loginfo("Ctrl+C pressed - exiting...")
                        break

                rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
    finally:
        # Cleanup
        if publisher is not None:
            # Only disarm if the vehicle is actually armed
            if publisher.current_state.armed:
                rospy.loginfo("Force disarming vehicle for safety")
                publisher.disarm_vehicle(force=True)
            publisher.stop_publishing()
        rospy.loginfo("Shutting down...")


if __name__ == '__main__':
    main()
