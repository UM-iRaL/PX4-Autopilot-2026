#include "lee_position_controller.hpp"

#include <cstdio>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

// Physical constants
namespace {
  // Gravity constant (m/s²)
  constexpr double kGravity = 9.81;

  // SolidWorks inertia matrix at COM in SW coordinate system (kg·m²)
  constexpr double kJswXX = 0.04520;
  constexpr double kJswXY = 0.0000;
  constexpr double kJswXZ = 0.000;
  constexpr double kJswYY = 0.08889;
  constexpr double kJswYZ = 0.0000;
  constexpr double kJswZZ = 0.12601;

  // Vehicle mass (kg)
  constexpr double kVehicleMass = 4.6;

  // Rotor thrust coefficient: Thrust = ct * omega^2 (N·s²/rad²)
  // This represents the thrust for ONE actuator channel (total from coaxial motor pair if applicable)
  // Determined from motor thrust stand tests using scripts/thrust_pwm_fit.py
  // Use kCt_total if testing with coaxial motors, kCt_per_motor for single motor
  constexpr double ct = 4.2068e-6;

  // Gimbal lock avoidance null-space vector gain
  constexpr double servo_range = 3; // How many revolutions we consider for both positive and negative directions
  constexpr double servo_response_factor = 1.0; // This highly depends on the update rate and the gains of the servo driver
  constexpr double l = 0.1905; // lx
  constexpr double b = 0.2032; // ly
  constexpr double nullspace_cancelation_factor_pitch = 0.5;
  constexpr double nullspace_cancelation_factor_roll = 0.5;

  const double r = sqrt(l*l + b*b); // TODO: This is not right :: jiawei: it looks right to me(?)
  const matrix::Vector3d k_hat(0.0, 0.0, 1.0);
  const matrix::Vector3d t1_psi_hat(-b/r, l/r, 0);
  const matrix::Vector3d t2_psi_hat(b/r, l/r, 0);
  const matrix::Vector3d t3_psi_hat(b/r, -l/r, 0);
  const matrix::Vector3d t4_psi_hat(-b/r, -l/r, 0);
  /* NEW : Adding a Null space component to the thrust distribution to avoid gimbal lock */
  // Linear combinations of any number of the six null space vectors to the thrusts will not affect total drone-frame wrench
  // Define nullspace vectors for each thruster (rows represent x, y, z components)
  // Nullspace basis: each column is the nullspace vector for one thruster
  // Split into x and z components for independent weighting
  const matrix::Vector3d nullspace_x1(-1, 0, 0);
  const matrix::Vector3d nullspace_z1(0, 0, -1);
  const matrix::Vector3d nullspace_x2(-1, 0, 0);
  const matrix::Vector3d nullspace_z2(0, 0, 1);
  const matrix::Vector3d nullspace_x3(1, 0, 0);
  const matrix::Vector3d nullspace_z3(0, 0, -1);
  const matrix::Vector3d nullspace_x4(1, 0, 0);
  const matrix::Vector3d nullspace_z4(0, 0, 1);
  const matrix::Vector3d nullspace_y1(0, -1, 0);
  const matrix::Vector3d nullspace_y2(0, 1, 0);
  const matrix::Vector3d nullspace_y3(0, -1, 0);
  const matrix::Vector3d nullspace_y4(0, 1, 0);
}

// Constructs a Lee position controller instance.
// Initializes the controller with default parameters and sets up initial state.
LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false),
      debug_output_(false) {
  InitializeParameters();
}

// Destroys the Lee position controller instance.
// Performs cleanup of controller resources.
LeePositionController::~LeePositionController() {}

// Initializes controller parameters with default values.
// Sets vehicle mass, inertia matrix, and marks parameters as initialized.
void LeePositionController::InitializeParameters() {
  // SolidWorks inertia @ COM in SW output CS (kg·m^2)
  matrix::SquareMatrix<double, 3> J_sw;
  J_sw(0,0)=kJswXX; J_sw(0,1)=kJswXY; J_sw(0,2)=kJswXZ;
  J_sw(1,0)=kJswXY; J_sw(1,1)=kJswYY; J_sw(1,2)=kJswYZ;
  J_sw(2,0)=kJswXZ; J_sw(2,1)=kJswYZ; J_sw(2,2)=kJswZZ;

  inertia_matrix = J_sw;

  vehicle_mass = kVehicleMass;
  initialized_params_ = true;
}

// Sets the current vehicle odometry state.
// Updates the controller with current position, orientation, velocity and angular velocity.
// `odometry` - Current vehicle state information including position, orientation and velocities.
void LeePositionController::SetOdometry(const DrewOdometry& odometry) {
  odometry_ = odometry;
}

// Gets the current vehicle odometry state.
// Returns the most recently set odometry information.
// Returns DrewOdometry struct containing current vehicle state.
DrewOdometry LeePositionController::GetOdometry() {
  return odometry_;
}

// Sets the desired trajectory point for the controller to track.
// Activates the controller and updates the target trajectory with desired position, velocity, 
// acceleration, yaw, and yaw rate.
// `command_trajectory` - Target trajectory containing desired position, velocity, acceleration and yaw.
void LeePositionController::SetTrajectoryPoint(
    const DrewTrajPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

// Computes element-wise product of two 3D vectors.
// Multiplies corresponding elements of vectors `a` and `b`.
// `a` - First input vector.
// `b` - Second input vector.
// Returns Vector3d containing element-wise products.
matrix::Vector3d cwiseProduct(const matrix::Vector3d& v1, const matrix::Vector3d& v2){
  return matrix::Vector3d({v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2)});
}

// Extracts a vector from its skew-symmetric matrix representation.
// Converts a 3x3 skew-symmetric matrix back to the original 3D vector.
// `skew_matrix` - Input 3x3 skew-symmetric matrix.
// `vector` - Output vector extracted from the skew matrix. Must not be null.
void vectorFromSkewMatrix(matrix::Matrix3d& skew_matrix, matrix::Vector3d* vector) {
  (*vector)(0) = skew_matrix(2, 1);
  (*vector)(1) = skew_matrix(0, 2);
  (*vector)(2) = skew_matrix(1, 0);
}

// Computes tilt angles for a single tilting rotor.
// Calculates alpha (roll tilt) and beta (pitch tilt) from desired thrust vector.
// Uses spherical coordinate conversion.
// `t` - Desired thrust vector for the rotor in body frame.
// `alpha` - Output roll tilt angle in radians.
// `beta` - Output pitch tilt angle in radians.
// `servo_memory` - Servo memory for continuous angle tracking.
void computeSingleAlphaBeta(const matrix::Vector3d& t,
                            double& alpha, double& beta,
                            ServoMemory& servo_memory)
{
  const double thrust = t.norm();

  // Exit if thrust is negligible.
  if (thrust <= 1e-6) {
    alpha = servo_memory.have_prev ? servo_memory.alpha_prev : 0.0;
    beta = servo_memory.have_prev ? servo_memory.beta_prev : 0.0;
    return;
  }

  const matrix::Vector3d t_hat = t / thrust;
  const double tix = t_hat(0);
  const double tiy = t_hat(1);
  const double tiz = t_hat(2);

  // Compute valid argument for asin(beta) = -tiy, clamped to [-1, 1]
  const double beta_base = asin(math::constrain(-tiy, -1.0, 1.0));
  // Calculate the base solution for alpha from tan(alpha) = tix / tiz
  const double alpha_base = atan2(tix, tiz);

    if (!servo_memory.have_prev) {
    alpha = alpha_base;
    beta  = beta_base;
  } else {
    const double alpha_prev = servo_memory.alpha_prev;
    // const double beta_prev  = servo_memory.beta_prev;

    const int k_count = 2 * servo_range + 1; // k = -servo_range ... +servo_range

    double best_alpha = alpha_base;
    double best_beta  = beta_base;
    double best_cost  = DBL_MAX;
    int best_branch = 1; // Track which branch: 1 or 2

    // First loop: Find best alpha and record which branch
    for (int i = 0; i < k_count; ++i) {
      const int k = i - servo_range;
      const double two_pi_k = 2.0 * M_PI * k;

      // Branch 1: alpha_base
      {
        const double a1 = alpha_base + two_pi_k;
        const double cost1 = fabs(a1 - alpha_prev);
        // PX4_INFO("Alpha Branch 1 [k=%d]: cost1=%.6f, a1=%.6f", k, cost1, a1);
        if (cost1 < best_cost) {
          best_cost  = cost1;
          best_alpha = a1;
          best_branch = 1;
        }
      }

      // Branch 2: alpha_base + pi
      {
        const double a2 = alpha_base + M_PI + two_pi_k;
        const double cost2 = fabs(a2 - alpha_prev);
        // PX4_INFO("Alpha Branch 2 [k=%d]: cost2=%.6f, a2=%.6f", k, cost2, a2);
        if (cost2 < best_cost) {
          best_cost  = cost2;
          best_alpha = a2;
          best_branch = 2;
        }
      }
    }

    // Second loop: Find best beta using the same branch
    best_cost = DBL_MAX;
    for (int i = 0; i < k_count; ++i) {
      const int k = i - servo_range;
      const double two_pi_k = 2.0 * M_PI * k;

      // Use the same branch that was selected for alpha
      double beta_candidate;
      if (best_branch == 1) {
        beta_candidate = beta_base + two_pi_k;
      } else {
        beta_candidate = M_PI - beta_base + two_pi_k;
      }

      const double cost = fabs(beta_candidate - servo_memory.beta_prev);
      // PX4_INFO("Beta [k=%d]: cost=%.6f, beta_candidate=%.6f", k, cost, beta_candidate);
      if (cost < best_cost) {
        best_cost = cost;
        best_beta = beta_candidate;
      }
    }

    // PX4_INFO("AlphaBeta Final best: cost=%.6f, alpha=%.6f, beta=%.6f", best_cost, best_alpha, best_beta);

    alpha = best_alpha;
    beta  = best_beta;
  }

  // Model the first-order response of the servos for the next iteration.
  // The 'alpha_prev' and 'beta_prev' values now represent the servo's estimated
  // current state, which moves towards the newly commanded target (alpha, beta)
  // at a rate determined by the response factor.
  if (servo_memory.have_prev) {
    servo_memory.alpha_prev += servo_response_factor * (alpha - servo_memory.alpha_prev);
    servo_memory.beta_prev += servo_response_factor * (beta - servo_memory.beta_prev);
  } else {
    servo_memory.alpha_prev = alpha;
    servo_memory.beta_prev = beta;
  }
  servo_memory.have_prev = true;
}

// Computes activation factor for null-space gain with dead zone.
// Creates a piecewise linear activation function:
// - Dead zone [-threshold, threshold]: returns 0
// - Above threshold: linearly grows from 0 (at threshold) to 1 (at 1.0)
// - Below -threshold: linearly grows from 0 (at -threshold) to -1 (at -1.0)
// `dot_product` - Dot product between thrust unit vector and body y-axis.
// `threshold` - Threshold value for dead zone activation.
// Returns activation factor to multiply with kNullVectorGain.
double computeNullSpaceActivation(double dot_product, double threshold) {
  if (dot_product > threshold) {
    // Positive activation: scale from 0 to 1 as dot_product goes from threshold to 1.0
    return (dot_product - threshold) / (1.0 - threshold);
  } else if (dot_product < -threshold) {
    // Negative activation: scale from 0 to -1 as dot_product goes from -threshold to -1.0
    return (dot_product + threshold) / (1.0 - threshold);
  } else {
    // Dead zone: no activation
    return 0.0;
  }
}

// Computes tilt angles and rotor speeds for all four rotors of a tilting quadcopter.
// Distributes desired body force and torque among four tilting rotors
// using geometric relationships.
// Assumes fixed quadcopter geometry with rotors at specified positions.
// New version that attempts to avoid gimbal lock by
// adding a pre-computed null space component to the thrust distribution.
// iB, jB, kB are the body frame axes (unit vectors).
// `force` - Desired total force vector in body frame (N).
// `T` - Desired total torque vector in body frame (N·m).
// `alphas` - Output array of roll tilt angles for each rotor (radians).
// `betas` - Output array of pitch tilt angles for each rotor (radians).
// `thrusts` - Output array of rotor thrusts (N).
void LeePositionController::computeAlphasBetasThrustsGimbalFree(
  const matrix::Vector3d& force, const matrix::Vector3d& T,
  matrix::Vector<double, 4>& alphas, matrix::Vector<double, 4>& betas,
  matrix::Vector<double, 4>& thrusts)
{
  double Tx = T(0), Ty = T(1), Tz = T(2);
  matrix::Vector3d jB(0.0, 1.0, 0.0); // Drone y-axis unit vector
  matrix::Vector3d iB(1.0, 0.0, 0.0); // Drone x-axis unit vector

  // === Base pseudo-inverse solution (particular solution) ===
  matrix::Vector3d t1 = force / 4 + (Tx / (4 * b)) * k_hat - (Ty / (4 * l)) * k_hat + (Tz / (4 * r)) * t1_psi_hat;
  matrix::Vector3d t2 = force / 4 - (Tx / (4 * b)) * k_hat - (Ty / (4 * l)) * k_hat + (Tz / (4 * r)) * t2_psi_hat;
  matrix::Vector3d t3 = force / 4 - (Tx / (4 * b)) * k_hat + (Ty / (4 * l)) * k_hat + (Tz / (4 * r)) * t3_psi_hat;
  matrix::Vector3d t4 = force / 4 + (Tx / (4 * b)) * k_hat + (Ty / (4 * l)) * k_hat + (Tz / (4 * r)) * t4_psi_hat;
  matrix::Vector3d* thrusters[4] = {&t1, &t2, &t3, &t4}; // grouping together...

  // Calculate individual eta values for each thruster
  double eta = 0.0;
  double gamma = 0.0;
  double dot_products_w_jB[4];
  double dot_products_w_iB[4];
  double thrust_with_eta_max = 0.0;
  double thrust_with_gamma_max = 0.0;
  for(int i=0; i<4; i++){
    dot_products_w_jB[i] = thrusters[i]->unit() * jB;
    dot_products_w_iB[i] = thrusters[i]->unit() * iB;
    double thrust_mag = thrusters[i]->norm();
    if(fabs(dot_products_w_jB[i]) > fabs(eta)) {
      eta = dot_products_w_jB[i];
      thrust_with_eta_max = thrust_mag;
    }
    if(fabs(dot_products_w_iB[i]) > fabs(gamma)) {
      gamma = dot_products_w_iB[i];
      thrust_with_gamma_max = thrust_mag;
    }
  }

  // Apply nullspace adjustment to each thruster with separate thresholds for x and z
  double eta_x = computeNullSpaceActivation(eta, 0.5) * thrust_with_eta_max * nullspace_cancelation_factor_roll;
  double eta_z = computeNullSpaceActivation(eta, 0.6) * thrust_with_eta_max * nullspace_cancelation_factor_roll;
  
  gamma = computeNullSpaceActivation(gamma, 0.7) * thrust_with_gamma_max * nullspace_cancelation_factor_pitch;
  double gamma_12 = 0.0;
  double gamma_34 = 0.0;
  if(gamma > 0){
    gamma_12 = gamma;
  } else {
    gamma_34 = gamma;
  }

  matrix::Vector3d t1_adjusted = t1 + nullspace_x1 * eta_x + nullspace_z1 * eta_z + nullspace_y1 * gamma_12;
  matrix::Vector3d t2_adjusted = t2 + nullspace_x2 * eta_x + nullspace_z2 * eta_z + nullspace_y2 * gamma_12;
  matrix::Vector3d t3_adjusted = t3 + nullspace_x3 * eta_x + nullspace_z3 * eta_z + nullspace_y3 * gamma_34;
  matrix::Vector3d t4_adjusted = t4 + nullspace_x4 * eta_x + nullspace_z4 * eta_z + nullspace_y4 * gamma_34;

  // === Compute alphas, betas ===
  computeSingleAlphaBeta(t1_adjusted, alphas(0), betas(0), servo_memories.servo_memory1);
  computeSingleAlphaBeta(t2_adjusted, alphas(1), betas(1), servo_memories.servo_memory2);
  computeSingleAlphaBeta(t3_adjusted, alphas(2), betas(2), servo_memories.servo_memory3);
  computeSingleAlphaBeta(t4_adjusted, alphas(3), betas(3), servo_memories.servo_memory4);

  // === Store thrust magnitudes directly ===
  thrusts(0) = t1_adjusted.norm();
  thrusts(1) = t2_adjusted.norm();
  thrusts(2) = t3_adjusted.norm();
  thrusts(3) = t4_adjusted.norm();
}

// Computes the desired body acceleration to track the trajectory setpoint.
// Uses PID control law based on position and velocity errors relative to the desired trajectory.
// Includes gravity compensation and feedforward acceleration from trajectory.
// `body_acceleration` - Output desired acceleration in body frame (m/s²). Must not be null.
void LeePositionController::ComputeDesiredBodyAcceleration(matrix::Vector3d* body_acceleration) {
  assert(body_acceleration);

  // Update delta_t
  const hrt_abstime now = hrt_absolute_time();
  delta_t = (now - _last_run) / 1e6;
  _last_run = now;

  matrix::Vector3d position_error;
  position_error = command_trajectory_.position_W - odometry_.position;
  position_error_int += position_error*delta_t;
  for(int i=0; i<3; i++){
    position_error_int(i) = math::constrain(position_error_int(i), -controller_parameters_.position_integral_lim_(i), controller_parameters_.position_integral_lim_(i));
  }
  
  matrix::Vector3d velocity_W = odometry_.velocity;
  matrix::Vector3d velocity_error = command_trajectory_.velocity_W - velocity_W;

  const matrix::Vector3d e3({0, 0, 1});

  // Compute base control acceleration (PID + gravity compensation + trajectory feedforward)
  matrix::Vector3d acceleration_W = (cwiseProduct(position_error, controller_parameters_.position_gain_)
                                    + cwiseProduct(velocity_error, controller_parameters_.velocity_gain_)
                                    + cwiseProduct(position_error_int, controller_parameters_.position_integral_gain_)) / vehicle_mass
                                    - kGravity * e3
                                    + command_trajectory_.acceleration_W;

  matrix::Dcm<double> R_body_to_world(odometry_.orientation);
  *body_acceleration = R_body_to_world.transpose() * acceleration_W;
}


// Computes the desired control torque to track attitude setpoint.
// Implements geometric attitude control with gyroscopic compensation using Euler's equation.
// Returns torque directly without requiring matrix multiplication by caller.
// `torque` - Output desired torque in body frame (N·m). Must not be null.
void LeePositionController::ComputeDesiredTorque(matrix::Vector3d* torque) {
  assert(torque);

  matrix::Dcm<double> R_body_to_world(odometry_.orientation);

  // Desired orientation from quaternion setpoint
  matrix::Dcm<double> R_des(command_trajectory_.orientation);

  // Compute attitude error from rotation matrices
  matrix::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R_body_to_world - R_body_to_world.transpose() * R_des);
  matrix::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);
  angular_error_int += angle_error*delta_t;
  for(int i=0; i<3; i++){
    angular_error_int(i) = math::constrain(angular_error_int(i), -controller_parameters_.angular_integral_lim_(i), controller_parameters_.angular_integral_lim_(i));
  }

  // Angular rate error using angular velocity setpoint (already in body frame)
  matrix::Vector3d angular_rate_error = odometry_.angular_velocity - (R_body_to_world.transpose() * R_des * command_trajectory_.angular_velocity_B);

  // Currently assuming Ω̇_d = 0 (no angular acceleration setpoint available)
  // TODO: Add angular acceleration setpoints to DrewTrajPoint if needed for aggressive maneuvers
  matrix::Vector3d angular_accel_des(0.0, 0.0, 0.0);  // Ω̇_d = 0 for now

  // Total torque: M = -k_R*e_R - k_Ω*e_Ω + Ω × JΩ - J* (Ω̂R^T*R_d*Ω_d) - R^T*R_d*Ω̇_d
  *torque = - cwiseProduct(angle_error, controller_parameters_.attitude_gain_)
            - cwiseProduct(angular_rate_error, controller_parameters_.angular_rate_gain_)
            - cwiseProduct(angular_error_int, controller_parameters_.angular_integral_gain_)
            + odometry_.angular_velocity % (inertia_matrix * odometry_.angular_velocity)
            - inertia_matrix *  (odometry_.angular_velocity % (R_body_to_world.transpose() * R_des * command_trajectory_.angular_velocity_B))
            - R_body_to_world.transpose() * R_des * angular_accel_des;
}
