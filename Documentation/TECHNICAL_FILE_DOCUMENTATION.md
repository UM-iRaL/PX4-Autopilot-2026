# Technical File Documentation - Morphable Drone Project

## Overview

This document provides detailed technical descriptions of the key files in the Morphable Drone project, explaining their functionality, interfaces, and role in the overall system architecture.

---

## Core Control System Files

### **1. `src/modules/geometric_control/geometric_control.cpp`**
**Main Control Loop & Motor/Servo Coordination**

**Purpose**: Primary control module that integrates SE(3) geometric control with hardware actuators (motors and tilting servos).

**Key Functionality**:
- **Control Loop**: Runs at high frequency, subscribes to odometry and trajectory setpoints
- **Frame Transformations**: Converts between PX4 coordinate frames and custom geometric controller frames
  ```cpp
  // Critical frame conversion (major breakthrough fix)
  matrix::Dcm<double> R_PX4_to_ours = matrix::Dcm<double>(matrix::Eulerf(0, 0, M_PI));
  matrix::Vector3d force_vector_des_ours = R_PX4_to_ours * force_vector_des;
  ```
- **Alpha/Beta Computation**: Calls `computeAlphasBetasOmegas()` to convert force/torque to tilt angles
- **Motor Control**: Maps rotor speeds to actuator controls with careful indexing
- **Servo Control**: Converts alpha/beta angles to Dynamixel position commands with precision mapping

**Recent Evolution**:
- **April 2025**: Updated motor mapping from named indices to direct array access
- **Motor Assignment**: 
  ```cpp
  actuators.control[0] = rotor_cmds(1, 0);  // Motor 2 -> Control 0
  actuators.control[1] = rotor_cmds(2, 0);  // Motor 3 -> Control 1  
  actuators.control[2] = rotor_cmds(3, 0);  // Motor 4 -> Control 2
  actuators.control[3] = rotor_cmds(0, 0);  // Motor 1 -> Control 3
  ```
- **Servo Mapping**: Complex alpha/beta to Dynamixel index mapping for physical servo positions

**Key Parameters**:
- Counter-based initialization (waits 4000 cycles before full operation)
- Conversion rate: `4096/(2π)` for radians to Dynamixel position units
- Test trajectory: `(1, 1, -1)` position setpoint

---

### **2. `src/modules/geometric_control/GeometricControl/lee_position_controller.cpp`**
**SE(3) Geometric Controller Implementation**

**Purpose**: Implements the core geometric control algorithm based on Lee et al.'s SE(3) controller.

**Key Algorithms**:

**A. Position Control**:
```cpp
void ComputeDesiredBodyAcceleration(matrix::Vector3d* body_acceleration)
```
- Computes desired acceleration in body frame using position and velocity errors
- Uses PID-style control with configurable gains
- Transforms from world frame to body frame

**B. Attitude Control**:
```cpp  
void ComputeDesiredAngularAcc(matrix::Vector3d* angular_acceleration)
```
- Implements SE(3) attitude control with rotation matrix error
- Uses skew-symmetric matrix operations for attitude error computation
- Applies attitude and angular rate gains

**C. Alpha/Beta/Omega Computation**:
```cpp
void computeAlphasBetasOmegas(const matrix::Vector3d& force, const matrix::Vector3d& T,
                              matrix::Vector<double, 4>& alphas, matrix::Vector<double, 4>& betas, 
                              matrix::Vector<double, 4>& omegas)
```

**Critical Algorithm**: Converts 6-DOF force/torque commands to individual rotor force vectors and tilt angles.

**Physical Constants** (Hardware-Specific):
```cpp
double ct = 8.54858e-06;  // Thrust coefficient  
double l = 0.18844;       // X-arm length
double b = 0.12869 + 0.04045;  // Y-arm length + offset
double r = sqrt(l*l + b*b);    // Diagonal distance
```

**Rotor Force Distribution**:
- Distributes total force/torque among 4 rotors based on geometric arrangement
- Computes individual thrust vectors: `t1, t2, t3, t4`
- Solves for tilt angles (alpha, beta) and rotor speeds (omega) per rotor

**Controller Gains**:
```cpp
static const matrix::Vector3d kDefaultPositionGain = fact*Vector3d(6, 6, 6);
static const matrix::Vector3d kDefaultVelocityGain = fact*Vector3d(4.7, 4.7, 4.7);  
static const matrix::Vector3d kDefaultAttitudeGain = Vector3d(3, 3, 3);
static const matrix::Vector3d kDefaultAngularRateGain = Vector3d(3, 3, 3);
```

---

### **3. `src/modules/geometric_control/GeometricControl/lee_position_controller.hpp`**
**Controller Class Definition & Data Structures**

**Key Data Structures**:

**A. Custom Odometry Structure**:
```cpp
struct DrewOdometry {
    matrix::Vector3d position;
    matrix::Quaterniond orientation;  
    matrix::Vector3d velocity;        // Body frame
    matrix::Vector3d angular_velocity;
};
```

**B. Trajectory Point Structure**:
```cpp
struct DrewTrajPoint {
    matrix::Vector3d position_W;      // World frame position
    matrix::Vector3d velocity_W;      // World frame velocity  
    matrix::Vector3d acceleration_W;  // World frame acceleration
    double yaw, yaw_rate;
};
```

**Class Interface**:
- **Initialization**: `InitializeParameters()` sets up gains and vehicle parameters
- **State Updates**: `SetOdometry()`, `SetTrajectoryPoint()` for external inputs
- **Control Computation**: Main computation methods for forces and torques
- **Hardware Interface**: `computeAlphasBetasOmegas()` for actuator commands

---

### **4. `src/modules/geometric_control/geometric_control.hpp`**
**Main Module Header & Parameter Definitions**

**Key Features**:
- **PX4 Integration**: Inherits from `ModuleBase` and `ModuleParams`
- **uORB Subscriptions**: All sensor and setpoint topic subscriptions
- **Publications**: Actuator control and servo command publishers
- **Parameter Framework**: Extensive parameter definitions for tuning

**Critical Parameters**:
```cpp
(ParamFloat<px4::params::CA_SV_TL0_MINA>) _param_tilt_min_angle,
(ParamFloat<px4::params::CA_SV_TL0_MAXA>) _param_tilt_max_angle,  
(ParamInt<px4::params::CA_TILTING_TYPE>) _param_tilting_type,      // 0:h-tilting, 1:omni
(ParamInt<px4::params::CA_AIRFRAME>) _param_airframe,             // 11: tilting_multirotors
```

**uORB Topics**:
- **Inputs**: `vehicle_local_position`, `vehicle_odometry`, `vehicle_angular_velocity`
- **Outputs**: `actuator_controls`, `dynamixel_controls`, `vehicle_thrust_setpoint`

---

## Hardware Driver Files

### **5. `src/drivers/dynamixel/dynamixel_driver.cpp`**
**Servo Hardware Interface**

**Purpose**: Low-level driver for Dynamixel servo communication via UART.

**Key Functionality**:

**A. Servo Communication Protocol**:
- **UART Configuration**: `/dev/ttyS1` at specific baud rate with proper termios settings
- **Packet Protocol**: Implements Dynamixel 2.0 protocol with CRC checking
- **Command Formats**: Position commands, configuration commands, status reads

**B. Servo Management**:
```cpp
void configure_servos(void)  // Sets position control mode, enables torque
void reboot(uint8_t id)      // Hardware reset for reliability  
void send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
```

**C. Reliability Features**:
- **Auto-Reboot**: Reboots servos 10 times before configuration (added Sept 2024)
- **Error Handling**: Packet stuffing, CRC validation, timeout handling
- **Position Control**: Extended position mode (`OPMODE_EXT_POS_CONTROL`)

**Recent Fixes**:
- **Integer Bug Fix**: Changed from `uint16_t` to `int32_t` for position commands
- **Timing Optimizations**: Removed excessive delays and debug output
- **Communication Reliability**: Added reboot sequence for initialization

**Critical Code**:
```cpp
// Position command with debugging
PX4_INFO("Sending servo %d a value of %d", i, (int)dynamixel_command.dynamixel_controls[i]);
send_command(i+1, REG_GOAL_POSITION, dynamixel_command.dynamixel_controls[i], 4);
```

---

### **6. `src/drivers/dynamixel/dynamixel_driver.h`**  
**Driver Interface Definition**

**Key Constants**:
```cpp
#define NUM_SERVOS 8          // Maximum servo count
#define SER_PORT "/dev/ttyS1" // UART port assignment  
#define REBOOT_COUNT 10       // Reliability reboot cycles
```

**Class Structure**:
- **Serial Communication**: UART configuration and packet handling
- **Servo State Tracking**: Detection, configuration, and reboot counters
- **Parameter Integration**: PX4 parameter system integration
- **Timing Management**: Precise timing for servo communication

---

## Control Allocation Files

### **7. `src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessTiltingMultirotor.hpp`**
**Tilting Multirotor Effectiveness Matrix**

**Purpose**: Computes actuator effectiveness matrices for tilting multirotors, handling the complex relationship between rotor thrust, tilt angles, and resulting forces/torques.

**Key Features**:

**A. Tilting Types**:
```cpp
int numMatrices() const override { return _tilting_type == 0 ? 1 : 2; }
```
- **Type 0**: H-tilting (1 effectiveness matrix)
- **Type 1**: Omnidirectional tilting (2 matrices for different control phases)

**B. Allocation Methods**:
- Uses `SEQUENTIAL_DESATURATION` for both tilting types
- Normalizes RPY channels for stability
- Handles multiple servo configurations (up to 5 servos)

**C. Servo Parameter Management**:
```cpp
struct ServoParam {
    float angle_min;  // Minimum tilt angle
    float angle_max;  // Maximum tilt angle  
};
```

**Integration Points**:
- **Rotor Effectiveness**: `ActuatorEffectivenessRotors` for thrust generation
- **Tilt Effectiveness**: `ActuatorEffectivenessTilts` for servo control
- **Parameter System**: Dynamic servo configuration via PX4 parameters

---

## Message Definition Files

### **8. `msg/dynamixel_controls.msg`**
**Servo Command Message**

```cpp
uint64 timestamp              # Command timestamp
int32[8] dynamixel_controls   # 8 servo position commands
```

**Usage**: 
- **Precision**: 32-bit signed integers for precise position control
- **Range**: Typically 0-4095 (12-bit) but supports extended range
- **Conversion**: Radians to servo units via `(angle * 4096)/(2π)`
- **Mapping**: Complex mapping from alpha/beta angles to physical servo indices

---

### **9. `msg/tilting_servo_sp.msg`** 
**Servo Setpoint Message**

```cpp
uint64 timestamp       # Command timestamp
uint8 NUM_SERVO_OUTPUTS = 4
float32[4] angle      # Servo angles in radians
```

**Purpose**: Higher-level servo angle commands (unused in current implementation, superseded by direct Dynamixel control).

---

### **10. `msg/tilting_mc_desired_angles.msg`**
**Desired Body Angles Message**

```cpp
uint64 timestamp      # Command timestamp  
float32 roll_body     # Desired roll angle in NED frame
float32 pitch_body    # Desired pitch angle in NED frame
```

**Purpose**: Intermediate message for desired body attitude (part of earlier tilting architecture).

---

## Airframe Configuration Files

### **11. `ROMFS/px4fmu_common/init.d-posix/airframes/25003_NDT_tilting`**
**NDT Tilting Drone Configuration**

**Key Configuration**:
```bash
param set-default SYS_CTRL_ALLOC 1     # Enable dynamic allocation
param set-default CA_AIRFRAME 11       # Tilting Multirotor  
param set-default CA_TILTING_TYPE 1    # Omnidirectional tilting
param set-default CA_ROTOR_COUNT 8     # 8 rotors
param set-default CA_SV_TL_COUNT 4     # 4 tilt servos
```

**Rotor Configuration**: Detailed X-Y-Z positions and moment coefficients for 8 rotors in symmetric configuration.

**PWM Mapping**: 
- **MAIN1-8**: Motor control (ESCs)
- **MAIN9-12**: Servo control (tilting mechanisms)

**Servo Limits**:
```bash
param set-default CA_SV_TL0_MAXA 30   # ±30° tilt limits
param set-default CA_SV_TL0_MINA -30  
```

---

## Development & Analysis Tools

### **12. `odom_plotter.py`**
**Odometry Analysis Tool**

**Purpose**: Parses flight log data and generates comprehensive odometry plots.

**Key Features**:
- **Data Parsing**: Extracts position, velocity, orientation, and angular rates
- **Quaternion Conversion**: Converts quaternions to Euler angles (roll, pitch, yaw)
- **Multi-Plot Display**: 4 subplot layout showing:
  1. **Position**: X, Y, Z coordinates over time
  2. **Linear Velocity**: VX, VY, VZ over time  
  3. **Orientation**: Roll, pitch, yaw in degrees
  4. **Angular Velocity**: Roll rate, pitch rate, yaw rate

**Usage**: 
```bash
python3 odom_plotter.py  # Reads from "out.txt"
```

**Data Format**: Parses custom log format with timestamp, position vectors, quaternions, and velocities.

---

### **13. `servo_plotter.py`**
**Servo Performance Analysis**

**Purpose**: Analyzes Dynamixel servo commands and visualizes servo behavior over time.

**Key Features**:
- **Command Extraction**: Parses servo position commands from log files
- **Unit Conversion**: Converts from servo units to degrees (`* 360/4096`)
- **Color Coding**: 
  - **Blue**: Alpha servos (indices 0, 3, 5, 7)
  - **Red**: Beta servos (indices 1, 2, 4, 6)
- **Performance Tracking**: Shows servo response and command patterns

**Analysis Capability**:
- Identifies servo mapping issues
- Validates alpha/beta command generation
- Monitors servo response characteristics

---

## System Integration & Data Flow

### **Control Flow Summary**:

1. **Odometry Input** → `geometric_control.cpp` main loop
2. **Trajectory Setpoint** → Lee position controller
3. **Force/Torque Computation** → `ComputeDesiredBodyAcceleration()` + `ComputeDesiredAngularAcc()`
4. **Alpha/Beta Generation** → `computeAlphasBetasOmegas()`
5. **Motor Commands** → Actuator control topics
6. **Servo Commands** → Dynamixel driver → Physical servos

### **Key Interfaces**:

- **PX4 ↔ Geometric Control**: uORB topics for sensor data and commands
- **Control ↔ Hardware**: Custom message types for servo control  
- **Allocation ↔ Effectiveness**: Matrix computations for actuator mapping
- **Driver ↔ Servos**: UART protocol for precision position control

### **Configuration Dependencies**:

- **Airframe Type**: Must be set to 11 (tilting multirotor)
- **Tilting Type**: 0 (H-tilting) or 1 (omnidirectional)  
- **Servo Limits**: Physical constraints for safe operation
- **Rotor Geometry**: Precise arm lengths and positions for effectiveness matrices

---

## Development Notes & Future Work

### **Critical Success Factors**:
1. **Frame Conversions**: Proper coordinate transformations between PX4 and custom frames
2. **Servo Reliability**: Robust communication with hardware reset capabilities
3. **Precise Mapping**: Correct alpha/beta to physical servo assignments
4. **Timing Coordination**: Synchronized motor and servo commands

### **Known Limitations**:
1. **Magic Numbers**: Several hardcoded physical constants need parameterization
2. **Test Trajectories**: Currently uses fixed test setpoints instead of dynamic mission planning
3. **Error Handling**: Limited fault tolerance for servo communication failures

### **Extension Points**:
1. **Mission Integration**: Connect to PX4 mission system for autonomous flight
2. **Parameter Tuning**: Implement adaptive gain scheduling
3. **Fault Detection**: Add servo failure detection and recovery
4. **Performance Optimization**: Optimize computational efficiency for higher control rates

This technical documentation provides the foundation for new developers to understand and extend the morphable drone control system effectively.