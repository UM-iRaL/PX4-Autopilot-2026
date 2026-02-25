/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file l1_adaptive_controller.cpp
 *
 * L1 Adaptive Control Augmentation Implementation
 *
 * @author Jose (IRAL Lab)
 */

#include "l1_adaptive_controller.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/math/Functions.hpp>
#include <cmath>

// Physical constants
namespace {
	constexpr double kGravity = 9.81;  // m/s²
}

// L1 Adaptive Control Safety Limits
namespace {
	// Maximum prediction error before constraining (prevents numerical instability)
	constexpr double max_v_error = 5.0;      // m/s - Max velocity prediction error allowed
	constexpr double max_omega_error = 1.5;  // rad/s - Max angular velocity prediction error allowed

	// Maximum uncertainty estimates (prevents runaway adaptation)
	constexpr double max_sigma_force = 50.0;   // N - Max estimated force uncertainty magnitude
	constexpr double max_sigma_moment = 10.0;  // N·m - Max estimated moment uncertainty magnitude

	// Maximum adaptive control outputs (final safety bounds)
	constexpr double max_adaptive_force = 50.0;  // N - Max adaptive force correction
	constexpr double max_adaptive_moment = 10.0;  // N·m - Max adaptive moment correction
}

// Constructor
L1AdaptiveController::L1AdaptiveController()
	: last_update_time_(0),
	  dt_(0.0),
	  v_hat_prev_(0.0, 0.0, 0.0),
	  omega_hat_prev_(0.0, 0.0, 0.0),
	  v_prev_(0.0, 0.0, 0.0),
	  omega_prev_(0.0, 0.0, 0.0),
	  force_nominal_prev_(0.0, 0.0, 0.0),
	  moment_nominal_prev_(0.0, 0.0, 0.0),
	  force_adaptive_prev_(0.0, 0.0, 0.0),
	  moment_adaptive_prev_(0.0, 0.0, 0.0),
	  sigma_force_hat_prev_(0.0, 0.0, 0.0),
	  sigma_moment_hat_prev_(0.0, 0.0, 0.0),
	  h_v_prev_(0.0, 0.0, 0.0),
	  h_omega_prev_(0.0, 0.0, 0.0),
	  lpf1_force_prev_(0.0, 0.0, 0.0),
	  lpf1_moment_prev_(0.0, 0.0, 0.0),
	  lpf2_moment_prev_(0.0, 0.0, 0.0),
	  initialized_(false),
	  inertia_cached_(false)
{
	R_prev_.setIdentity();
	inertia_matrix_cached_.setZero();
	inertia_inverse_cached_.setZero();

	// Default parameters - very conservative for safety
	params_.as_v = -10.0;
	params_.as_omega = -10.0;
	params_.cutoff_force = 5.0;
	params_.cutoff_moment_1 = 1.5;
	params_.cutoff_moment_2 = 0.5;
	params_.enable = 0;  // Master switch disabled by default
	params_.enable_force = 0;  // Force channel disabled by default
	params_.enable_moment = 0;  // Moment channel disabled by default
}

// Destructor
L1AdaptiveController::~L1AdaptiveController() {}

// Reset controller state
void L1AdaptiveController::Reset()
{
	v_hat_prev_.zero();
	omega_hat_prev_.zero();
	v_prev_.zero();
	omega_prev_.zero();
	force_nominal_prev_.zero();
	moment_nominal_prev_.zero();
	force_adaptive_prev_.zero();
	moment_adaptive_prev_.zero();
	sigma_force_hat_prev_.zero();
	sigma_moment_hat_prev_.zero();
	h_v_prev_.zero();
	h_omega_prev_.zero();
	lpf1_force_prev_.zero();
	lpf1_moment_prev_.zero();
	lpf2_moment_prev_.zero();
	R_prev_.setIdentity();
	initialized_ = false;
	inertia_cached_ = false;
	last_update_time_ = 0;

	PX4_INFO("L1 Adaptive Controller: State reset");
}

// Update parameters
void L1AdaptiveController::UpdateParameters(const L1AdaptiveParameters& params)
{
	// Detect if any enable flag is transitioning from 0 to 1
	// params_ holds the old values, params holds the new incoming values
	// Check BEFORE updating params_ to compare old vs new
	bool enable_transition = (params_.enable == 0 && params.enable == 1) ||
	                         (params_.enable_force == 0 && params.enable_force == 1) ||
	                         (params_.enable_moment == 0 && params.enable_moment == 1);

	// Reset controller if any parameter is being enabled to start with clean state
	if (enable_transition && initialized_) {
		PX4_INFO("L1 Adaptive Controller: Parameter enabled, resetting controller state");
		Reset();
	}

	params_ = params;

	PX4_INFO("L1 Adaptive Controller parameters updated:");
	PX4_INFO("To enable or disable the controller, set the 'L1AD_ACTIVE', 'L1AD_FORCE_EN', and 'L1AD_MOMENT_EN' parameters.");
	PX4_INFO("  Master Enable: %d", params_.enable);
	PX4_INFO("  Force Enable: %d", params_.enable_force);
	PX4_INFO("  Moment Enable: %d", params_.enable_moment);
	PX4_INFO("  As_v: %.2f", params_.as_v);
	PX4_INFO("  As_omega: %.2f", params_.as_omega);
	PX4_INFO("  Cutoff_force: %.2f Hz", params_.cutoff_force);
	PX4_INFO("  Cutoff_moment_1: %.2f Hz", params_.cutoff_moment_1);
	PX4_INFO("  Cutoff_moment_2: %.2f Hz", params_.cutoff_moment_2);
}

// Initialize state predictor on first run
void L1AdaptiveController::InitializeStatePredictor(const DrewOdometry& odometry,
	const matrix::Vector3d& force_nominal,
	const matrix::Vector3d& moment_nominal)
{
	v_prev_ = odometry.velocity;
	omega_prev_ = odometry.angular_velocity;
	v_hat_prev_ = odometry.velocity;
	omega_hat_prev_ = odometry.angular_velocity;
	R_prev_ = matrix::Dcm<double>(odometry.orientation);

	// Initialize previous control inputs to current values to avoid zero-input transient
	force_nominal_prev_ = force_nominal;
	moment_nominal_prev_ = moment_nominal;
	force_adaptive_prev_.zero();
	moment_adaptive_prev_.zero();

	initialized_ = true;
	last_update_time_ = hrt_absolute_time();

	PX4_INFO("L1 Adaptive Controller: State predictor initialized");
}

// Main computation function
void L1AdaptiveController::ComputeAdaptiveAugmentation(
	const DrewOdometry& odometry,
	const matrix::Vector3d& force_nominal,
	const matrix::Vector3d& moment_nominal,
	double vehicle_mass,
	const matrix::SquareMatrix<double, 3>& inertia_matrix,
	matrix::Vector3d* adaptive_force,
	matrix::Vector3d* adaptive_moment)
{
	// Check if L1 is enabled
	if (params_.enable == 0) {
		// If controller was running and is now disabled, reset state
		// This ensures clean initialization when re-enabled
		if (initialized_) {
			Reset();
		}
		adaptive_force->zero();
		adaptive_moment->zero();
		return;
	}

	// Initialize on first run
	if (!initialized_) {
		InitializeStatePredictor(odometry, force_nominal, moment_nominal);
		adaptive_force->zero();
		adaptive_moment->zero();
		return;
	}

	// Compute time step
	const hrt_abstime now = hrt_absolute_time();
	dt_ = (now - last_update_time_) / 1e6;  // Convert μs to seconds
	last_update_time_ = now;

	// Sanity check on dt
	if (dt_ <= 0.0 || dt_ > 0.1) {  // Max 100ms between updates
		PX4_WARN("L1: Invalid dt = %.6f s, skipping update", dt_);
		adaptive_force->zero();
		adaptive_moment->zero();
		return;
	}

	// Current state
	matrix::Vector3d v_now = odometry.velocity;
	matrix::Vector3d omega_now = odometry.angular_velocity;
	matrix::Dcm<double> R_body_to_ned(odometry.orientation);

	// Compute inertia inverse (cached for performance since inertia is constant)
	matrix::SquareMatrix<double, 3> inertia_inverse;
	if (!inertia_cached_ || inertia_matrix != inertia_matrix_cached_) {
		inertia_inverse = inertia_matrix.I();
		inertia_matrix_cached_ = inertia_matrix;
		inertia_inverse_cached_ = inertia_inverse;
		inertia_cached_ = true;
	} else {
		inertia_inverse = inertia_inverse_cached_;
	}

	// Step 1: Update state predictor with previous control inputs (both nominal and adaptive from k-1)
	// Per equation (19): predictor uses total control u_b(k-1) + u_ad(k-1)
	UpdateStatePredictor(force_nominal_prev_, moment_nominal_prev_, force_adaptive_prev_, moment_adaptive_prev_,
	                     vehicle_mass, inertia_matrix, inertia_inverse, R_prev_);

	// Step 2: Compute uncertainty estimates from prediction errors
	matrix::Vector3d sigma_force_hat, sigma_moment_hat;
	ComputeUncertaintyEstimates(v_now, omega_now, R_body_to_ned, vehicle_mass,
	                           inertia_matrix, &sigma_force_hat, &sigma_moment_hat);

	// Step 3: Apply low-pass filters to get adaptive control
	ApplyLowPassFilters(sigma_force_hat, sigma_moment_hat, adaptive_force,
	                   adaptive_moment, R_body_to_ned);

	// Step 3.5: Selectively disable force/moment channels based on individual enable flags
	// This allows enabling only moment control during takeoff, then enabling force later
	if (params_.enable_force == 0) {
		adaptive_force->zero();
	}
	if (params_.enable_moment == 0) {
		adaptive_moment->zero();
	}

	// Step 4: Store values for next iteration
	v_prev_ = v_now;
	omega_prev_ = omega_now;
	R_prev_ = R_body_to_ned;
	force_nominal_prev_ = force_nominal;
	moment_nominal_prev_ = moment_nominal;
	force_adaptive_prev_ = *adaptive_force;
	moment_adaptive_prev_ = *adaptive_moment;
	sigma_force_hat_prev_ = sigma_force_hat;
	sigma_moment_hat_prev_ = sigma_moment_hat;

	// Step 5: Log state for debugging
	// LogState(v_now, omega_now, sigma_force_hat, sigma_moment_hat,
	//         *adaptive_force, *adaptive_moment);
}

// Update state predictor
void L1AdaptiveController::UpdateStatePredictor(
	const matrix::Vector3d& force_total,
	const matrix::Vector3d& moment_total,
	const matrix::Vector3d& adaptive_force,
	const matrix::Vector3d& adaptive_moment,
	double vehicle_mass,
	const matrix::SquareMatrix<double, 3>& inertia_matrix,
	const matrix::SquareMatrix<double, 3>& inertia_inverse,
	const matrix::Dcm<double>& R_body_to_ned)
{
	// Velocity predictor (NED frame)
	// v_hat = v_hat_prev + (g*e3 + F_total/m + h(t) + As*z_tilde)*dt
	// Per equation (19) in the L1 adaptive control paper

	matrix::Vector3d e3(0.0, 0.0, 1.0);  // Down direction in NED
	matrix::Vector3d v_pred_error_prev = v_hat_prev_ - v_prev_;

	// Translational dynamics in NED frame
	// The geometric controller outputs force that represents the desired acceleration command
	// To get actual acceleration, we need to account for gravity:
	// a_actual = F_command/m - g*e3
	// where g*e3 = [0, 0, 9.81] is gravity pulling down in NED
	// A negative z-force (upward thrust) opposes positive z-gravity (downward pull)
	matrix::Vector3d accel_ned = kGravity * e3 
								+ (1/vehicle_mass)*R_body_to_ned * (force_total+adaptive_force)
								+ h_v_prev_
								+ v_pred_error_prev * params_.as_v;

	// State predictor with h(t) term and correction term (equation 19)
	// h_v represents the estimated uncertainty feedback in the predictor
	v_hat_prev_ = v_hat_prev_ + (accel_ned ) * dt_;

	// Angular velocity predictor (body frame)
	// omega_hat = omega_hat_prev + (J^-1 * (M_total - omega_hat x J*omega_hat) + h(t) + As*z_tilde)*dt
	// Per equation (19) in the L1 adaptive control paper

	matrix::Vector3d omega_pred_error_prev = omega_hat_prev_ - omega_prev_;

	// Rotational dynamics: J*omega_dot_hat = M - omega_hat x J*omega_hat
	// IMPORTANT: Use predicted omega_hat in gyroscopic term for consistency with state predictor
	matrix::Vector3d gyroscopic_term = omega_hat_prev_ % (inertia_matrix * omega_hat_prev_);
	matrix::Vector3d angular_accel = -inertia_inverse * gyroscopic_term + inertia_inverse * (moment_total+adaptive_moment)
									+ h_omega_prev_ 
									+ omega_pred_error_prev * params_.as_omega;

	// State predictor with h(t) term and correction term (equation 19)
	// h_omega represents the estimated uncertainty feedback in the predictor
	omega_hat_prev_ = omega_hat_prev_ + (angular_accel ) * dt_;
}

// Compute uncertainty estimates from prediction errors
void L1AdaptiveController::ComputeUncertaintyEstimates(
	const matrix::Vector3d& v_actual,
	const matrix::Vector3d& omega_actual,
	const matrix::Dcm<double>& R_body_to_ned,
	double vehicle_mass,
	const matrix::SquareMatrix<double, 3>& inertia_matrix,
	matrix::Vector3d* sigma_force_hat,
	matrix::Vector3d* sigma_moment_hat)
{
	// Compute prediction errors
	matrix::Vector3d v_pred_error = v_hat_prev_ - v_actual;
	matrix::Vector3d omega_pred_error = omega_hat_prev_ - omega_actual;

	// Constrain prediction errors to prevent numerical explosion
	v_pred_error(0) = math::constrain(v_pred_error(0), -max_v_error, max_v_error);
	v_pred_error(1) = math::constrain(v_pred_error(1), -max_v_error, max_v_error);
	v_pred_error(2) = math::constrain(v_pred_error(2), -max_v_error, max_v_error);
	omega_pred_error(0) = math::constrain(omega_pred_error(0), -max_omega_error, max_omega_error);
	omega_pred_error(1) = math::constrain(omega_pred_error(1), -max_omega_error, max_omega_error);
	omega_pred_error(2) = math::constrain(omega_pred_error(2), -max_omega_error, max_omega_error);

	// Exponential coefficients for state predictor poles
	double exp_as_v_dt = std::exp(params_.as_v * dt_);
	double exp_as_omega_dt = std::exp(params_.as_omega * dt_);

	// Compute denominators with numerical safeguard
	double denom_v = exp_as_v_dt - 1.0;
	double denom_omega = exp_as_omega_dt - 1.0;

	// For small dt or small |As|, use Taylor series approximation to avoid division by near-zero
	// (exp(As*dt) - 1) / (As*dt) ≈ 1 for small As*dt
	constexpr double min_denom = 1e-6;
	if (std::fabs(denom_v) < min_denom) {
		denom_v = (denom_v >= 0.0) ? min_denom : -min_denom;
	}
	if (std::fabs(denom_omega) < min_denom) {
		denom_omega = (denom_omega >= 0.0) ? min_denom : -min_denom;
	}

	// Compute Φ^(-1)μ = As·(exp(As*Ts) - I)^(-1)·exp(As*Ts)·z_tilde(k)
	matrix::Vector3d phi_inv_mu_v = v_pred_error * params_.as_v * exp_as_v_dt / denom_v;
	matrix::Vector3d phi_inv_mu_omega = omega_pred_error * params_.as_omega * exp_as_omega_dt / denom_omega;

	// Compute h(t) = -Φ^(-1)μ(iTs) from Equation (20)
	// This is the uncertainty estimation term from Algorithm 1, Step 3
	matrix::Vector3d h_v = -phi_inv_mu_v;
	matrix::Vector3d h_omega = -phi_inv_mu_omega;

	// Store h(t) for use in state predictor (equation 19)
	h_v_prev_ = h_v;
	h_omega_prev_ = h_omega;

	// Compute uncertainty estimates σ̂(k) = -B̂(R(k))^(-1)·Φ^(-1)μ(iTs) from Equation (21)
	// Since h = -Φ^(-1)μ, we have σ̂ = B̂^(-1)·h
	// For force: B̂^(-1) = m, so σ_F = m·h  (NED frame)
	*sigma_force_hat = -R_body_to_ned.transpose()* h_v * vehicle_mass;

	// For moment: B̂^(-1) = J, so σ_M = J·h  (body frame)
	*sigma_moment_hat = -inertia_matrix * h_omega;

	// Saturate uncertainty estimates to prevent runaway
	for (int i = 0; i < 3; i++) {
		(*sigma_force_hat)(i) = math::constrain((*sigma_force_hat)(i), -max_sigma_force, max_sigma_force);
		(*sigma_moment_hat)(i) = math::constrain((*sigma_moment_hat)(i), -max_sigma_moment, max_sigma_moment);
	}
}

// Apply low-pass filters to uncertainty estimates
void L1AdaptiveController::ApplyLowPassFilters(
	const matrix::Vector3d& sigma_force_hat,
	const matrix::Vector3d& sigma_moment_hat,
	matrix::Vector3d* force_adaptive,
	matrix::Vector3d* moment_adaptive,
	const matrix::Dcm<double>& R_body_to_ned)
{
	// Compute filter coefficients (first-order: y[k] = a1*y[k-1] + a2*u[k])
	// a1 = exp(-wc * dt), a2 = 1 - a1

	double omega_c_force = 2.0 * M_PI * params_.cutoff_force;
	double lpf1_coeff_force_a1 = std::exp(-omega_c_force * dt_);
	double lpf1_coeff_force_a2 = 1.0 - lpf1_coeff_force_a1;

	double omega_c_moment_1 = 2.0 * M_PI * params_.cutoff_moment_1;
	double lpf1_coeff_moment_a1 = std::exp(-omega_c_moment_1 * dt_);
	double lpf1_coeff_moment_a2 = 1.0 - lpf1_coeff_moment_a1;

	// Second filter coefficients (DISABLED - uncomment if re-enabling second filter)
	// double omega_c_moment_2 = 2.0 * M_PI * params_.cutoff_moment_2;
	// double lpf2_coeff_moment_a1 = std::exp(-omega_c_moment_2 * dt_);
	// double lpf2_coeff_moment_a2 = 1.0 - lpf2_coeff_moment_a1;

	// Apply first-order filter to force channel (single filter)
	// Force uncertainty is in NED frame, filter in NED frame
	matrix::Vector3d u_ad_force_ned = lpf1_coeff_force_a1 * lpf1_force_prev_
	                                   + lpf1_coeff_force_a2 * sigma_force_hat;
	lpf1_force_prev_ = u_ad_force_ned;

	// Apply first low-pass filter to moment channel (body frame)
	matrix::Vector3d u_ad_moment_int = lpf1_coeff_moment_a1 * lpf1_moment_prev_
	                                    + lpf1_coeff_moment_a2 * sigma_moment_hat;
	lpf1_moment_prev_ = u_ad_moment_int;

	// Second low-pass filter (DISABLED - uncomment to re-enable cascaded filtering)
	// matrix::Vector3d u_ad_moment_body = lpf2_coeff_moment_a1 * lpf2_moment_prev_
	//                                      + lpf2_coeff_moment_a2 * u_ad_moment_int;
	// lpf2_moment_prev_ = u_ad_moment_body;

	// Use first filter output directly (second filter disabled)
	matrix::Vector3d u_ad_moment_body = u_ad_moment_int;

	// Negate to get adaptive control (compensates for uncertainty)
	// Force is already in NED frame, moment is in body frame
	*force_adaptive = -u_ad_force_ned;
	*moment_adaptive = -u_ad_moment_body;

	// Final saturation on adaptive outputs to ensure bounded control
	for (int i = 0; i < 3; i++) {
		(*force_adaptive)(i) = math::constrain((*force_adaptive)(i), -max_adaptive_force, max_adaptive_force);
		(*moment_adaptive)(i) = math::constrain((*moment_adaptive)(i), -max_adaptive_moment, max_adaptive_moment);
	}

	// Check for NaN and reset if detected
	if (!PX4_ISFINITE((*force_adaptive)(0)) || !PX4_ISFINITE((*force_adaptive)(1)) ||
	    !PX4_ISFINITE((*force_adaptive)(2)) || !PX4_ISFINITE((*moment_adaptive)(0)) ||
	    !PX4_ISFINITE((*moment_adaptive)(1)) || !PX4_ISFINITE((*moment_adaptive)(2))) {
		PX4_INFO("L1 WARNING: NaN detected in adaptive outputs! Resetting controller.");
		PX4_INFO("  force_adaptive:  [%s, %s, %s]",
			PX4_ISFINITE((*force_adaptive)(0)) ? "OK" : "NaN",
			PX4_ISFINITE((*force_adaptive)(1)) ? "OK" : "NaN",
			PX4_ISFINITE((*force_adaptive)(2)) ? "OK" : "NaN");
		PX4_INFO("  moment_adaptive: [%s, %s, %s]",
			PX4_ISFINITE((*moment_adaptive)(0)) ? "OK" : "NaN",
			PX4_ISFINITE((*moment_adaptive)(1)) ? "OK" : "NaN",
			PX4_ISFINITE((*moment_adaptive)(2)) ? "OK" : "NaN");
		Reset();
		force_adaptive->zero();
		moment_adaptive->zero();
	}
}

// Log L1 adaptive controller state
void L1AdaptiveController::LogState(
	const matrix::Vector3d& v_actual,
	const matrix::Vector3d& omega_actual,
	const matrix::Vector3d& sigma_force_hat,
	const matrix::Vector3d& sigma_moment_hat,
	const matrix::Vector3d& force_adaptive,
	const matrix::Vector3d& moment_adaptive)
{
	// Log state predictions vs actual (every ~1 second to avoid spam)
	static hrt_abstime last_log_time = 0;
	const hrt_abstime now = hrt_absolute_time();

	if ((now - last_log_time) > 1000000) {  // 1 Hz logging
		// Prediction errors
		matrix::Vector3d v_error = v_hat_prev_ - v_actual;
		matrix::Vector3d omega_error = omega_hat_prev_ - omega_actual;

		PX4_INFO("L1: v_err=[%.3f, %.3f, %.3f] m/s",
		         (double)v_error(0), (double)v_error(1), (double)v_error(2));
		PX4_INFO("L1: ω_err=[%.3f, %.3f, %.3f] rad/s",
		         (double)omega_error(0), (double)omega_error(1), (double)omega_error(2));
		PX4_INFO("L1: σ_F=[%.2f, %.2f, %.2f] N",
		         (double)sigma_force_hat(0), (double)sigma_force_hat(1), (double)sigma_force_hat(2));
		PX4_INFO("L1: σ_M=[%.2f, %.2f, %.2f] N·m",
		         (double)sigma_moment_hat(0), (double)sigma_moment_hat(1), (double)sigma_moment_hat(2));
		PX4_INFO("L1: F_ad=[%.2f, %.2f, %.2f] N",
		         (double)force_adaptive(0), (double)force_adaptive(1), (double)force_adaptive(2));
		PX4_INFO("L1: M_ad=[%.2f, %.2f, %.2f] N·m",
		         (double)moment_adaptive(0), (double)moment_adaptive(1), (double)moment_adaptive(2));

		last_log_time = now;
	}
}
