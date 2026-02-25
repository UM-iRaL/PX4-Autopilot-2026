/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

#include <matrix/matrix/math.hpp>

#include <lib/mathlib/math/Limits.hpp>
#include <drivers/drv_hrt.h>
#include <cmath>
#include <math.h>

struct DrewOdometry {
  DrewOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(), // TODO: This might fail lol. It should be inited to the identity?
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0) {
          orientation.identity();
        };

  DrewOdometry(const matrix::Vector3d& _position,
                const matrix::Quaterniond& _orientation,
                const matrix::Vector3d& _velocity,
                const matrix::Vector3d& _angular_velocity) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
  };

  matrix::Vector3d position;
  matrix::Quaterniond orientation;
  matrix::Vector3d velocity; // Velocity is expressed in the Body frame!
  matrix::Vector3d angular_velocity;
};

struct ServoMemory {
  bool   have_prev = false; // Default member initializers (C++11)
  double alpha_prev = 0.0;
  double beta_prev = 0.0;
};

// Now you don't need to specify the values again!
struct ServoMemories {
  ServoMemory servo_memory1;
  ServoMemory servo_memory2;
  ServoMemory servo_memory3;
  ServoMemory servo_memory4;
};

struct DrewTrajPoint {
  DrewTrajPoint()
      : position_W(0.0, 0.0, 0.0),
        velocity_W(0.0, 0.0, 0.0),
        acceleration_W(0.0, 0.0, 0.0),
        angular_velocity_B(0.0, 0.0, 0.0) {
    orientation.identity();
  };

  DrewTrajPoint(const matrix::Vector3d& _position_W,
                const matrix::Vector3d& _velocity_W,
                const matrix::Vector3d& _acceleration_W,
                double _yaw,
                double _yaw_rate) {
    position_W = _position_W;
    velocity_W = _velocity_W;
    acceleration_W = _acceleration_W;
    // Create quaternion from yaw only (roll=0, pitch=0)
    matrix::Eulerd euler(0.0, 0.0, _yaw);
    orientation = matrix::Quaterniond(euler);
    // Angular velocity in body frame (only yaw rate)
    angular_velocity_B = matrix::Vector3d(0.0, 0.0, _yaw_rate);
  };

  // Constructor with full state (position, velocity, acceleration, full attitude, angular rates)
  DrewTrajPoint(const matrix::Vector3d& _position_W,
                const matrix::Vector3d& _velocity_W,
                const matrix::Vector3d& _acceleration_W,
                double _roll,
                double _pitch,
                double _yaw,
                double _roll_rate,
                double _pitch_rate,
                double _yaw_rate) {
    position_W = _position_W;
    velocity_W = _velocity_W;
    acceleration_W = _acceleration_W;
    // Create quaternion from Euler angles
    matrix::Eulerd euler(_roll, _pitch, _yaw);
    orientation = matrix::Quaterniond(euler);
    // Angular velocity in body frame
    angular_velocity_B = matrix::Vector3d(_roll_rate, _pitch_rate, _yaw_rate);
  };

  // Constructor with quaternion and angular velocity
  DrewTrajPoint(const matrix::Vector3d& _position_W,
                const matrix::Vector3d& _velocity_W,
                const matrix::Vector3d& _acceleration_W,
                const matrix::Quaterniond& _orientation,
                const matrix::Vector3d& _angular_velocity_B) {
    position_W = _position_W;
    velocity_W = _velocity_W;
    acceleration_W = _acceleration_W;
    orientation = _orientation;
    angular_velocity_B = _angular_velocity_B;
  };

  matrix::Vector3d position_W;
  matrix::Vector3d velocity_W;
  matrix::Vector3d acceleration_W;

  // Attitude setpoint (quaternion, world to body frame)
  matrix::Quaterniond orientation;

  // Angular velocity setpoint (rad/s, body frame)
  matrix::Vector3d angular_velocity_B;
};

// Default gains are now set from PX4 parameters in geometric_control.cpp:parameters_updated()
// These fallback values match the PX4 parameter defaults and are used only for initialization
static const matrix::Vector3d kDefaultPositionGain = matrix::Vector3d(0.95, 0.95, 1.0);  // MPC_XY_P, MPC_Z_P
static const matrix::Vector3d kDefaultVelocityGain = matrix::Vector3d(1.8, 1.8, 4.0);    // MPC_XY_VEL_P_ACC, MPC_Z_VEL_P_ACC
static const matrix::Vector3d kDefaultPositionIntegralGain = matrix::Vector3d(0.0, 0.0, 0.0); // MPC_XY_VEL_I_ACC, MPC_Z_VEL_I_ACC
static const matrix::Vector3d kDefaultAttitudeGain = matrix::Vector3d(6.5, 6.5, 2.8);    // MC_ROLL_P, MC_PITCH_P, MC_YAW_P
static const matrix::Vector3d kDefaultAngularRateGain = matrix::Vector3d(1.0, 1.0, 1.0); // MC_ROLLRATE_K, MC_PITCHRATE_K, MC_YAWRATE_K
static const matrix::Vector3d kDefaultAngularIntegralGain = matrix::Vector3d(0.0, 0.0, 0.0); // MC_ROLLRATE_I, MC_PITCHRATE_I, MC_YAWRATE_I
static const matrix::Vector3d positionIntegralLim(1.0, 1.0, 1.0);
static const matrix::Vector3d angularIntegralLim(0.0, 0.0, 0.0);


class LeePositionControllerParameters {
 public:
  LeePositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        position_integral_gain_(kDefaultPositionIntegralGain),
        position_integral_lim_(positionIntegralLim),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain),
        angular_integral_gain_(kDefaultAngularIntegralGain),
        angular_integral_lim_(angularIntegralLim) {
          
  }

  matrix::Vector3d position_gain_;
  matrix::Vector3d velocity_gain_;
  matrix::Vector3d position_integral_gain_;
  matrix::Vector3d position_integral_lim_;
  matrix::Vector3d attitude_gain_;
  matrix::Vector3d angular_rate_gain_;
  matrix::Vector3d angular_integral_gain_;
  matrix::Vector3d angular_integral_lim_;
};

class LeePositionController {
 public:
  LeePositionController();
  ~LeePositionController();
  void InitializeParameters();

  DrewOdometry GetOdometry();

  // void CalculateRotorVelocities(matrix::VectorXd* rotor_velocities) const;

  void SetOdometry(const DrewOdometry& odometry);
  void SetTrajectoryPoint(const DrewTrajPoint& command_trajectory);

  LeePositionControllerParameters controller_parameters_;
  // VehicleParameters vehicle_parameters_;

  void ComputeDesiredAngularAcc(matrix::Vector3d* angular_acceleration);
  void ComputeDesiredTorque(matrix::Vector3d* torque);
  void ComputeDesiredBodyAcceleration(matrix::Vector3d* body_acceleration);

  void computeAlphasBetasOmegas(const matrix::Vector3d& force, const matrix::Vector3d& T,
                  matrix::Vector<double, 4>& alphas, matrix::Vector<double, 4>& betas, matrix::Vector<double, 4>& omegas);
  
  void computeAlphasBetasOmegasGimbalFree(
    const matrix::Vector3d& force, const matrix::Vector3d& T,
    matrix::Vector<double, 4>& alphas, matrix::Vector<double, 4>& betas,
    matrix::Vector<double, 4>& omegas);

  void computeAlphasBetasThrustsGimbalFree(
    const matrix::Vector3d& force, const matrix::Vector3d& T,
    matrix::Vector<double, 4>& alphas, matrix::Vector<double, 4>& betas,
    matrix::Vector<double, 4>& thrusts);

  double vehicle_mass;
  matrix::SquareMatrix<double, 3> inertia_matrix;

 private:
  bool initialized_params_;
  bool controller_active_;
  bool debug_output_;

  matrix::Vector3d normalized_attitude_gain_;
  matrix::Vector3d normalized_angular_rate_gain_;
  // matrix::Matrix4d angular_acc_to_rotor_velocities_;

  DrewTrajPoint command_trajectory_;
  DrewOdometry odometry_;

  // Integral error terms and servo memory for control
  matrix::Vector3d angular_error_int{0.0, 0.0, 0.0};
  matrix::Vector3d position_error_int{0.0, 0.0, 0.0};
  ServoMemories servo_memories;
  double delta_t{0.0};
  hrt_abstime _last_run{0};


};

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H