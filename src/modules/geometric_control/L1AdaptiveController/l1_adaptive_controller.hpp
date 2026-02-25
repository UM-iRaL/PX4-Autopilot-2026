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
 * @file l1_adaptive_controller.hpp
 *
 * L1 Adaptive Control Augmentation for Omnidirectional Quadcopters
 *
 * This controller implements L1 adaptive augmentation for fully actuated
 * (omnidirectional) tilting quadcopters. It estimates and compensates for
 * matched uncertainties in all 6 DOF (force and moment channels).
 *
 * Based on the L1 adaptive control theory and adapted from ArduPilot implementation.
 *
 * @author Jose (IRAL Lab)
 */

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include "../GeometricControl/lee_position_controller.hpp"

/**
 * L1 Adaptive Controller Parameters
 */
struct L1AdaptiveParameters {
	double as_v;          // State predictor pole for velocity (negative value, e.g., -5.0)
	double as_omega;      // State predictor pole for angular velocity (negative value, e.g., -5.0)
	double cutoff_force;  // Low-pass filter cutoff frequency for force channel (Hz)
	double cutoff_moment_1;  // First low-pass filter cutoff for moment channel (Hz)
	double cutoff_moment_2;  // Second low-pass filter cutoff for moment channel (Hz)
	int enable;           // Master enable/disable flag (0 = disabled, 1 = enabled) - must be enabled for individual channels
	int enable_force;     // Enable adaptive force augmentation (0 = disabled, 1 = enabled)
	int enable_moment;    // Enable adaptive moment augmentation (0 = disabled, 1 = enabled)
};

/**
 * L1 Adaptive Controller Class
 *
 * Implements L1 adaptive augmentation to compensate for model uncertainties
 * and external disturbances. Works in NED frame for consistency with the
 * geometric controller.
 */
class L1AdaptiveController {
public:
	/**
	 * Constructor
	 */
	L1AdaptiveController();

	/**
	 * Destructor
	 */
	~L1AdaptiveController();

	/**
	 * Compute L1 adaptive augmentation
	 *
	 * @param odometry Current vehicle state (position, velocity, orientation, angular velocity)
	 * @param force_nominal Nominal force command from geometric controller (NED frame, N)
	 * @param moment_nominal Nominal moment command from geometric controller (body frame, N·m)
	 * @param vehicle_mass Vehicle mass (kg)
	 * @param inertia_matrix Vehicle inertia matrix (kg·m²)
	 * @param adaptive_force Output adaptive force augmentation (NED frame, N)
	 * @param adaptive_moment Output adaptive moment augmentation (body frame, N·m)
	 */
	void ComputeAdaptiveAugmentation(
		const DrewOdometry& odometry,
		const matrix::Vector3d& force_nominal,
		const matrix::Vector3d& moment_nominal,
		double vehicle_mass,
		const matrix::SquareMatrix<double, 3>& inertia_matrix,
		matrix::Vector3d* adaptive_force,
		matrix::Vector3d* adaptive_moment
	);

	/**
	 * Reset the L1 controller state
	 * Clears all state predictors, uncertainty estimates, and filter states
	 */
	void Reset();

	/**
	 * Update controller parameters
	 *
	 * @param params New parameter values
	 */
	void UpdateParameters(const L1AdaptiveParameters& params);

	/**
	 * Get current parameters
	 *
	 * @return Current parameter values
	 */
	L1AdaptiveParameters GetParameters() const { return params_; }

private:
	// Controller parameters
	L1AdaptiveParameters params_;

	// Timing
	hrt_abstime last_update_time_;  // Last update timestamp (μs)
	double dt_;                      // Time step (s)

	// State predictor variables (previous step values)
	matrix::Vector3d v_hat_prev_;      // Predicted velocity (NED frame, m/s)
	matrix::Vector3d omega_hat_prev_;  // Predicted angular velocity (body frame, rad/s)
	matrix::Vector3d v_prev_;          // Actual velocity from previous step (NED frame, m/s)
	matrix::Vector3d omega_prev_;      // Actual angular velocity from previous step (body frame, rad/s)
	matrix::Dcm<double> R_prev_;       // Rotation matrix from previous step (body to NED)

	// Control inputs from previous step (for state predictor)
	matrix::Vector3d force_nominal_prev_;   // Previous nominal force (NED frame, N)
	matrix::Vector3d moment_nominal_prev_;  // Previous nominal moment (body frame, N·m)
	matrix::Vector3d force_adaptive_prev_;  // Previous adaptive force (NED frame, N)
	matrix::Vector3d moment_adaptive_prev_; // Previous adaptive moment (body frame, N·m)

	// Uncertainty estimates (previous step)
	matrix::Vector3d sigma_force_hat_prev_;   // Estimated force uncertainty (NED frame, N)
	matrix::Vector3d sigma_moment_hat_prev_;  // Estimated moment uncertainty (body frame, N·m)

	// h(t) terms from L1 algorithm (equation 19) - represent estimated uncertainty feedback
	matrix::Vector3d h_v_prev_;      // h(t) for velocity predictor (NED frame, m/s²)
	matrix::Vector3d h_omega_prev_;  // h(t) for angular velocity predictor (body frame, rad/s²)

	// Low-pass filter states
	matrix::Vector3d lpf1_force_prev_;    // First filter state for force (NED frame)
	matrix::Vector3d lpf1_moment_prev_;   // First filter state for moment (body frame)
	matrix::Vector3d lpf2_moment_prev_;   // Second filter state for moment (body frame, cascaded)

	// Initialization flag
	bool initialized_;

	// Cached inertia matrix and its inverse (since inertia is constant)
	matrix::SquareMatrix<double, 3> inertia_matrix_cached_;
	matrix::SquareMatrix<double, 3> inertia_inverse_cached_;
	bool inertia_cached_;

	/**
	 * Initialize state predictor on first run
	 *
	 * @param odometry Current vehicle state
	 * @param force_nominal Initial nominal force command (NED frame, N)
	 * @param moment_nominal Initial nominal moment command (body frame, N·m)
	 */
	void InitializeStatePredictor(const DrewOdometry& odometry,
		const matrix::Vector3d& force_nominal,
		const matrix::Vector3d& moment_nominal);

	/**
	 * Update state predictor
	 *
	 * @param force_total Total force including adaptive term (NED frame, N)
	 * @param moment_total Total moment including adaptive term (body frame, N·m)
	 * @param vehicle_mass Vehicle mass (kg)
	 * @param inertia_matrix Vehicle inertia matrix (kg·m²)
	 * @param inertia_inverse Inverse of inertia matrix (m²/kg)
	 * @param R_body_to_ned Rotation matrix from body to NED frame
	 */
	void UpdateStatePredictor(
		const matrix::Vector3d& force_total,
		const matrix::Vector3d& moment_total,
		const matrix::Vector3d& adaptive_force,
		const matrix::Vector3d& adaptive_moment,
		double vehicle_mass,
		const matrix::SquareMatrix<double, 3>& inertia_matrix,
		const matrix::SquareMatrix<double, 3>& inertia_inverse,
		const matrix::Dcm<double>& R_body_to_ned
	);

	/**
	 * Compute uncertainty estimates from prediction errors
	 *
	 * @param v_actual Current actual velocity (NED frame, m/s)
	 * @param omega_actual Current actual angular velocity (body frame, rad/s)
	 * @param R_body_to_ned Current rotation matrix (body to NED)
	 * @param vehicle_mass Vehicle mass (kg)
	 * @param inertia_matrix Vehicle inertia matrix (kg·m²)
	 * @param sigma_force_hat Output estimated force uncertainty (NED frame, N)
	 * @param sigma_moment_hat Output estimated moment uncertainty (body frame, N·m)
	 */
	void ComputeUncertaintyEstimates(
		const matrix::Vector3d& v_actual,
		const matrix::Vector3d& omega_actual,
		const matrix::Dcm<double>& R_body_to_ned,
		double vehicle_mass,
		const matrix::SquareMatrix<double, 3>& inertia_matrix,
		matrix::Vector3d* sigma_force_hat,
		matrix::Vector3d* sigma_moment_hat
	);

	/**
	 * Apply low-pass filters to uncertainty estimates
	 *
	 * @param sigma_force_hat Estimated force uncertainty (NED frame, N)
	 * @param sigma_moment_hat Estimated moment uncertainty (body frame, N·m)
	 * @param force_adaptive Output filtered adaptive force (NED frame, N)
	 * @param moment_adaptive Output filtered adaptive moment (body frame, N·m)
	 * @param R_body_to_ned Rotation matrix from body to NED frame (unused after fix)
	 */
	void ApplyLowPassFilters(
		const matrix::Vector3d& sigma_force_hat,
		const matrix::Vector3d& sigma_moment_hat,
		matrix::Vector3d* force_adaptive,
		matrix::Vector3d* moment_adaptive,
		const matrix::Dcm<double>& R_body_to_ned
	);

	/**
	 * Log L1 adaptive controller state for debugging
	 *
	 * @param v_actual Current actual velocity (NED frame, m/s)
	 * @param omega_actual Current actual angular velocity (body frame, rad/s)
	 * @param sigma_force_hat Estimated force uncertainty (body frame, N)
	 * @param sigma_moment_hat Estimated moment uncertainty (body frame, N·m)
	 * @param force_adaptive Adaptive force augmentation (NED frame, N)
	 * @param moment_adaptive Adaptive moment augmentation (body frame, N·m)
	 */
	void LogState(
		const matrix::Vector3d& v_actual,
		const matrix::Vector3d& omega_actual,
		const matrix::Vector3d& sigma_force_hat,
		const matrix::Vector3d& sigma_moment_hat,
		const matrix::Vector3d& force_adaptive,
		const matrix::Vector3d& moment_adaptive
	);
};
