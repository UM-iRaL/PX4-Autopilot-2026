# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a PX4-based flight stack repository for controlling multicopters with morphable/tiltable rotors, developed by the IRAL lab. The repository implements a geometric controller on SE(3) for multicoptors and extends the work of Marcellini et al. to support tiltable quadcopters and multirotor configurations.

## Key Features

- **Morphable/Tiltable Rotor Control**: Specialized control systems for drones with tilting rotors
- **Geometric Control**: Implementation of SE(3) geometric controllers
- **Multiple Airframe Configurations**: Support for omnidirectional tilting drones, one-tilt drones, and various multirotor configurations
- **PX4 Integration**: Built on the PX4 flight stack with custom modules and message types

## Build System

The project uses a CMake-based build system with Make wrapper commands. The build system supports multiple target platforms including SITL (Software In The Loop) simulation and hardware targets.

### Common Build Commands

**Pixhawk Build:**
```bash
make px4_fmu-v6x_default
```

**SITL Simulation:**
```bash
# Morphable drone (recommended for all PX4 SITL experiments)
make px4_sitl gazebo_morphable

# Standard SITL build
make px4_sitl_default
```

**Testing:**
```bash
# Run all tests
make tests

# Run integration tests
make tests_integration

# Run with coverage
make tests_coverage
```

**Code Quality:**
```bash
# Check code formatting
make check_format

# Format code
make format

# Static analysis
make scan-build
make clang-tidy
```

**Cleanup:**
```bash
# Clean build artifacts
make clean

# Deep clean (including submodules)
make distclean
```

## Architecture Overview

### Core Directory Structure

- **`src/modules/`**: Flight control modules
  - `geometric_control/`: SE(3) geometric position and attitude control
  - `mc_att_control/`: Multicopter attitude control
  - `mc_pos_control/`: Multicopter position control  
  - `mc_rate_control/`: Multicopter rate control
  - `control_allocator/`: Control allocation for tilting multirotors
  - `vtol_att_control/`: VTOL attitude control with tilting support

- **`src/lib/`**: Shared libraries and utilities
  - `mixer/`: Control mixing for various airframe configurations
  - `matrix/`: Mathematical matrix operations
  - `systemlib/`: System-level utilities

- **`msg/`**: Custom message definitions
  - `tilting_servo_sp.msg`: Tilting servo setpoint message
  - `tilting_mc_desired_angles.msg`: Desired angles for tilting multicopter

- **`ROMFS/px4fmu_common/init.d-posix/airframes/`**: Airframe configurations
  - `25003_NDT_tilting`: NDT tilting drone configuration
  - `25004_baby_k`: Baby K drone configuration
  - `25001_generic_tilting_multirotor`: Generic tilting multirotor

### Key Modules

**Geometric Control (`src/modules/geometric_control/`)**
- Implements SE(3) geometric controller based on Lee et al.
- Handles position and attitude control for tilting multirotors
- Integrates with PX4's control allocation system
- Custom parameters for tilting angle limits and control modes

**Control Allocator (`src/modules/control_allocator/`)**
- `ActuatorEffectivenessTiltingMultirotor`: Effectiveness calculations for tilting rotors
- `ActuatorEffectivenessRotors`: Standard rotor effectiveness
- `ActuatorEffectivenessTilts`: Tilt servo effectiveness
- Supports omnidirectional and H-tilting configurations

**Tilting Airframe Support**
- Parameters: `CA_TILTING_TYPE` (0: h-tilting, 1: omnidirectional)
- Airframe type: `CA_AIRFRAME = 11` (Tilting Multirotor)
- Servo limits: `CA_SV_TL*_MINA/MAXA` for tilting angle constraints

## Development Setup

**Prerequisites:**
- Python 3 with packages from `Tools/setup/requirements.txt`
- CMake 3.2+
- Ninja build system (recommended)
- Gazebo simulator for SITL testing

**Python Dependencies:**
```bash
pip install -r Tools/setup/requirements.txt
```

**Key Python packages:** matplotlib, numpy, pymavlink, pyulog, jinja2, empy

## Simulation and Testing

**Gazebo Simulation:**
The repository includes custom Gazebo models for tilting drones. Use the airframe-specific make targets to launch simulation with the appropriate model.

**Troubleshooting Gazebo Submodule Issues:**
If you encounter issues with the `Tools/sitl_gazebo` submodule:
```bash
cd Tools/sitl_gazebo
git fetch
git checkout iral
cd ../..
make px4_sitl gazebo_morphable
```

For persistent submodule issues:
```bash
git submodule sync --recursive
git submodule update --init --recursive --force
```


**Running SITL with ROS 1 (MAVROS):**

**Method 1: Single Launch File (Recommended for ROS sensor integration)**

This method provides full ROS integration including camera topics and is the recommended approach when using ROS sensors.

1. **Setup ROS environment** (add to `~/.bashrc` for convenience):
   ```bash
   source /opt/ros/noetic/setup.bash
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/Morphable-Drone:$HOME/Morphable-Drone/Tools/sitl_gazebo
   ```

2. **Build PX4 SITL** (required once to build Gazebo plugins):
   ```bash
   cd ~/Morphable-Drone
   make px4_sitl gazebo_morphable
   ```

3. **Launch everything with roslaunch**:
   ```bash
   roslaunch launch/mavros_posix_sitl.launch vehicle:=morphable
   ```

   This single command launches:
   - PX4 SITL flight controller
   - Gazebo simulator with ROS integration
   - MAVROS (MAVLink-ROS bridge)
   - Camera and sensor ROS topics (e.g., `/camera/fisheye1/image_raw`)

4. **Run control scripts** (in a separate terminal):
   ```bash
   python scripts/mavros_full_state_setpoint_publisher.py
   ```

**Method 2: Two-Terminal Method (PX4 control only, no ROS sensor plugins)**

This method works for MAVROS control but does NOT provide ROS sensor topics (cameras, etc.) because Gazebo runs without ROS integration.

1. Start PX4 SITL simulation:
   ```bash
   make px4_sitl gazebo_morphable
   ```
2. In a new terminal, launch MAVROS:
   ```bash
   source /opt/ros/noetic/setup.bash
   roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
   ```
3. Run control scripts:
   ```bash
   python scripts/mavros_full_state_setpoint_publisher.py
   ```

**Important:** Use Method 1 (roslaunch) when you need camera topics or other ROS sensor plugins. Use Method 2 only for PX4 control without ROS sensors.

**MAVROS Integration:**
For external trajectory following:
1. Set parameter `COM_RCL_EXCEPT = 4` in QGroundControl for offboard mode
2. Use MAVROS for external control scripts
3. Reference `Documentation/ExternalTrajectoryFollowing.md` for examples

## Parameters and Configuration

**Key Parameters:**
- `CA_TILTING_TYPE`: Tilting configuration (0=H-tilting, 1=omnidirectional)
- `CA_AIRFRAME`: Set to 11 for tilting multirotor
- `CA_SV_TL*_MINA/MAXA`: Tilting angle limits (degrees)
- `CA_ROTOR_COUNT`: Number of rotors
- `CA_SV_TL_COUNT`: Number of tilt servos
- `MC_PITCH_ON_TILT`: Enable pitch control via tilting

**Rotor Configuration:**
Rotor positions and orientations are defined via `CA_ROTOR*_PX/PY/PZ/KM` parameters, where:
- PX/PY/PZ: Position in body frame
- KM: Moment coefficient (positive for CCW, negative for CW)

## Working with Tilting Multirotors

When developing for tilting multirotors:

1. **Airframe Selection**: Use appropriate airframe files (25001, 25003, 25004, etc.)
2. **Control Allocation**: Ensure proper configuration in `control_allocator` module
3. **Message Types**: Utilize custom messages like `tilting_servo_sp` for servo control
4. **Parameter Tuning**: Configure tilting limits and control parameters appropriately
5. **Simulation**: Test with Gazebo models designed for tilting configurations

## Testing Strategy

- Unit tests for control algorithms in individual modules
- Integration tests with MAVSDK test framework
- SITL simulation for validation
- Hardware-in-the-loop testing for final validation
- Coverage analysis available via make targets

## Python Scripts Architecture

### Trajectory Module (`scripts/trajectory/`)

The trajectory module provides predefined flight trajectories with a class hierarchy design:

```
scripts/trajectory/
├── __init__.py                 # Package exports
├── base.py                     # Abstract base classes
├── trajectory_manager.py       # Main orchestrator
├── wait_trajectory.py          # Time-scaled path following
├── stateless/                  # Pure time-based trajectories
│   ├── angular_sweep.py        # Roll/pitch/yaw sweeps (trajectories 1-3, 8)
│   ├── circular_motion.py      # Circular paths (trajectories 5, 11)
│   └── combined_rotation.py    # Multi-axis rotation (trajectory 4)
└── stateful/                   # Trajectories with state/resources
    ├── takeoff.py              # Trajectory 6
    ├── landing.py              # Trajectory 7
    ├── hand_tracking.py        # Trajectory 9 (vision-based)
    └── wait_wrapper.py         # Trajectory 10 (configurable path)
```

**Class Hierarchy:**
- `BaseTrajectory` (abstract): Common utilities (`build_setpoint()`, `make_quaternion_y_axis()`)
- `StatelessTrajectory`: Pure functions of time, no internal state
- `StatefulTrajectory`: Trajectories requiring initialization/cleanup (`on_start()`/`on_stop()`)

**Adding a New Trajectory:**
1. Create a class inheriting from `StatelessTrajectory` or `StatefulTrajectory`
2. Implement `get_setpoint(self, t: float) -> Dict` returning position, velocity, orientation, rates
3. Override `on_start()`/`on_stop()` if lifecycle management needed
4. Add instance to `TrajectoryManager._trajectories` dict with a new ID

**Trajectory IDs:**
| ID | Class | Description |
|----|-------|-------------|
| 1-3 | `AngularSweepTrajectory` | Roll, pitch, yaw sweeps |
| 4 | `CombinedRotationTrajectory` | Multi-axis oscillation |
| 5 | `CircularMotionTrajectory` | XY circle with yaw |
| 6 | `TakeoffTrajectory` | S-curve takeoff with L1 controller |
| 7 | `LandingTrajectory` | Attitude stabilization + descent |
| 8 | `AngularSweepTrajectory` | Pitch two-step (up-pause-down) |
| 9 | `HandTrackingTrajectory` | Vision-based attitude control |
| 10 | `WaitTrajectoryWrapper` | Configurable path following |
| 11 | `CircularMotionTrajectory` | YZ circle with pitch toward center |

**Key Design Patterns:**
- Parameterized classes reduce duplication (e.g., `AngularSweepTrajectory(axis='roll')`)
- `build_setpoint()` standardizes return dict construction
- `on_start()`/`on_stop()` hooks handle resource lifecycle
- `TrajectoryManager` delegates to trajectory instances, not methods

### Other Script Modules

- **`scripts/config/`**: `SetpointConfig` - loads YAML configuration
- **`scripts/control/`**: `MAVROSFullStatePublisher` - publishes setpoints via MAVROS
- **`scripts/utils/`**: Math utilities (quaternions, angle wrapping), keyboard handler
- **`scripts/vision/`**: `HandTracker` - MediaPipe hand detection for trajectory 9
- **`scripts/trajectory_loader.py`**: Loads waypoint trajectories from YAML