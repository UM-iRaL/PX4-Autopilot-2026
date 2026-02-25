/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "geometric_control.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

// Motor and actuator control constants
namespace {
	// Motor command mapping parameters
	constexpr float kUidle    = 0.05f;     // Idle throttle floor

	// Dynamixel servo conversion
	constexpr double kDynamixelConversionRate = 4096.0 / (2.0 * M_PI);  // 651.917873627 (encoder counts per radian)
}

ModuleBase::Descriptor GeometricPositionControl::desc{task_spawn, custom_command, print_usage};

// Constructs a geometric position control module instance.
// Initializes the controller for use with morphable/tilting multirotor vehicles.
// `vtol` - Whether this is a VTOL vehicle (currently unused).
GeometricPositionControl::GeometricPositionControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

// Destroys the geometric position control module instance.
// Cleans up allocated performance counters.
GeometricPositionControl::~GeometricPositionControl()
{
	perf_free(_loop_perf);
}

// Initializes the geometric control module.
// Registers callbacks for vehicle angular velocity updates and sets initial trajectory setpoint.
// Returns true if initialization succeeds, false otherwise.
bool GeometricPositionControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

// Updates controller parameters from the parameter system.
// Loads rate control gains (for RateControl class) and geometric controller gains from PX4 parameters.
// The geometric controller gains are loaded from standard PX4 parameters:
// - Position: MPC_XY_P, MPC_Z_P
// - Velocity: MPC_XY_VEL_P_ACC, MPC_Z_VEL_P_ACC
// - Attitude: MC_ROLL_P, MC_PITCH_P, MC_YAW_P
// - Angular Rate: MC_ROLLRATE_K, MC_PITCHRATE_K, MC_YAWRATE_K
void GeometricPositionControl::parameters_updated()
{
	// Update all parameters from the parameter system
	updateParams();

	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setPidGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_roll_i.get(), _param_mc_pitch_i.get(), _param_mc_yaw_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_r_int_lim.get(), _param_mc_p_int_lim.get(), _param_mc_y_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));


	// Geometric controller gains from PX4 parameters
	_geometric_control.controller_parameters_.position_gain_ = matrix::Vector3d(
		(double)_param_mpc_xy_p.get(),
		(double)_param_mpc_xy_p.get(),
		(double)_param_mpc_z_p.get()
	);

	_geometric_control.controller_parameters_.velocity_gain_ = matrix::Vector3d(
		(double)_param_mpc_xy_vel_p.get(),
		(double)_param_mpc_xy_vel_p.get(),
		(double)_param_mpc_z_vel_p.get()
	);

	_geometric_control.controller_parameters_.position_integral_gain_ = matrix::Vector3d(
		(double)_param_mpc_xy_i.get(),
		(double)_param_mpc_xy_i.get(),
		(double)_param_mpc_z_i.get()
	);

	_geometric_control.controller_parameters_.attitude_gain_ = matrix::Vector3d(
		(double)_param_mc_roll_p.get(),
		(double)_param_mc_pitch_p.get(),
		(double)_param_mc_yaw_p.get()
	);

	_geometric_control.controller_parameters_.angular_rate_gain_ = matrix::Vector3d(
		(double)_param_mc_rollrate_k.get(),
		(double)_param_mc_pitchrate_k.get(),
		(double)_param_mc_yawrate_k.get()
	);
	_geometric_control.controller_parameters_.angular_integral_gain_ = matrix::Vector3d(
		(double)_param_mc_roll_i.get(),
		(double)_param_mc_pitch_i.get(),
		(double)_param_mc_yaw_i.get()
	);
	_geometric_control.controller_parameters_.angular_integral_lim_ = matrix::Vector3d(
		(double)_param_mc_r_int_lim.get(),
		(double)_param_mc_p_int_lim.get(),
		(double)_param_mc_y_int_lim.get()
	);

	// Print updated gains
	PX4_INFO("Geometric controller gains updated:");
	PX4_INFO("  Position: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.position_gain_(0),
		(double)_geometric_control.controller_parameters_.position_gain_(1),
		(double)_geometric_control.controller_parameters_.position_gain_(2));
	PX4_INFO("  Velocity: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.velocity_gain_(0),
		(double)_geometric_control.controller_parameters_.velocity_gain_(1),
		(double)_geometric_control.controller_parameters_.velocity_gain_(2));
	PX4_INFO("  Position Integral: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.position_integral_gain_(0),
		(double)_geometric_control.controller_parameters_.position_integral_gain_(1),
		(double)_geometric_control.controller_parameters_.position_integral_gain_(2));
	PX4_INFO("  Attitude: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.attitude_gain_(0),
		(double)_geometric_control.controller_parameters_.attitude_gain_(1),
		(double)_geometric_control.controller_parameters_.attitude_gain_(2));
	PX4_INFO("  Angular Rate: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.angular_rate_gain_(0),
		(double)_geometric_control.controller_parameters_.angular_rate_gain_(1),
		(double)_geometric_control.controller_parameters_.angular_rate_gain_(2));
	PX4_INFO("  Angular Integral: [%.2f, %.2f, %.2f]",
		(double)_geometric_control.controller_parameters_.angular_integral_gain_(0),
		(double)_geometric_control.controller_parameters_.angular_integral_gain_(1),
		(double)_geometric_control.controller_parameters_.angular_integral_gain_(2));

	// L1 Adaptive Controller parameters
	L1AdaptiveParameters l1_params;
	l1_params.enable = _param_l1_enable.get();
	l1_params.enable_force = _param_l1_enable_force.get();
	l1_params.enable_moment = _param_l1_enable_moment.get();
	l1_params.as_v = static_cast<double>(_param_l1_as_v.get());
	l1_params.as_omega = static_cast<double>(_param_l1_as_omega.get());
	l1_params.cutoff_force = static_cast<double>(_param_l1_cutoff_f.get());
	l1_params.cutoff_moment_1 = static_cast<double>(_param_l1_cutoff_m1.get());
	l1_params.cutoff_moment_2 = static_cast<double>(_param_l1_cutoff_m2.get());
	_l1_adaptive.UpdateParameters(l1_params);
}

// Main control loop execution function.
// Processes trajectory setpoints, vehicle odometry, computes control commands, and publishes
// actuator commands for motors and tilting servos. Runs at approximately 250Hz.
void GeometricPositionControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// Check for parameter updates from QGroundControl
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		parameters_updated();
	}

	// Check if mixer is ready before sending commands
	// Wait for mixer to initialize on first run
	static hrt_abstime first_run_time = 0;
	static constexpr hrt_abstime MIXER_INIT_DELAY = 200000; // 200ms in microseconds

	if (first_run_time == 0) {
		first_run_time = hrt_absolute_time();
	}

	if (!_mixer_ready) {
		if (hrt_elapsed_time(&first_run_time) >= MIXER_INIT_DELAY) {
			_mixer_ready = true;
			PX4_INFO("Mixer initialization delay complete, starting control output");
		} else {
			// Mixer not ready yet, skip this cycle
			return;
		}
	}

	// Storage for full state setpoint
	static matrix::Vector3d local_position_des = {0.0, 0.0, 0.0};
	static matrix::Vector3d local_velocity_des = {0.0, 0.0, 0.0};
	static matrix::Vector3d local_accel_des = {0.0, 0.0, 0.0};
	static matrix::Quaterniond orientation_des(1.0, 0.0, 0.0, 0.0);  // Initialize to identity quaternion (w, x, y, z)
	static matrix::Vector3d angular_velocity_des = {0.0, 0.0, 0.0};

	bool setpoint_updated = false;

	// Process position/velocity/acceleration setpoint from MAVROS
	vehicle_local_position_setpoint_s trajectory_setpoint;
	if (_trajectory_setpoint_sub.update(&trajectory_setpoint)){
		if (!isnan(trajectory_setpoint.x) && !isnan(trajectory_setpoint.y) && !isnan(trajectory_setpoint.z)){
			// Position setpoints are now used directly from the trajectory_setpoint message
			local_position_des = {(double)trajectory_setpoint.x, (double)trajectory_setpoint.y, (double)trajectory_setpoint.z};

			// Sanitize velocity and acceleration fields
			local_velocity_des = {
				isnan(trajectory_setpoint.vx) ? 0.0 : (double)trajectory_setpoint.vx,
				isnan(trajectory_setpoint.vy) ? 0.0 : (double)trajectory_setpoint.vy,
				isnan(trajectory_setpoint.vz) ? 0.0 : (double)trajectory_setpoint.vz
			};

			local_accel_des = {
				isnan(trajectory_setpoint.acceleration[0]) ? 0.0 : (double)trajectory_setpoint.acceleration[0],
				isnan(trajectory_setpoint.acceleration[1]) ? 0.0 : (double)trajectory_setpoint.acceleration[1],
				isnan(trajectory_setpoint.acceleration[2]) ? 0.0 : (double)trajectory_setpoint.acceleration[2]
			};
			
			setpoint_updated = true;
		}
	}

	// Process attitude setpoint (quaternion) from MAVROS
	vehicle_attitude_setpoint_s attitude_sp;
	if (_attitude_setpoint_sub.update(&attitude_sp)) {
		// In Body-NED Frame
		// Extract quaternion directly
		matrix::Quatf q_f(attitude_sp.q_d);
		orientation_des(0) = (double)q_f(0);
		orientation_des(1) = (double)q_f(1);
		orientation_des(2) = (double)q_f(2);
		orientation_des(3) = (double)q_f(3);

		setpoint_updated = true;
	}

	// Process body rate setpoint (angular velocity in body frame) from MAVROS
	vehicle_rates_setpoint_s rates_sp;
	if (_rates_setpoint_sub.update(&rates_sp)) {
		// In Body-NED Frame
		angular_velocity_des(0) = (double)rates_sp.roll;
		angular_velocity_des(1) = (double)rates_sp.pitch;
		angular_velocity_des(2) = (double)rates_sp.yaw;

		setpoint_updated = true;
	}

	// Send the full state setpoint to the geometric controller
	if (setpoint_updated) {
		// In Body-NED Frame
		DrewTrajPoint traj_point(
			local_position_des,
			local_velocity_des,
			local_accel_des,
			orientation_des,
			angular_velocity_des
		);
		_geometric_control.SetTrajectoryPoint(traj_point);
	}

	// Set the Odometry for the geometric controller (~40 Hz). TODO: Are other updates faster maybe??
	vehicle_odometry_s curr_odom;
	if (_vehicle_odometry_sub.update(&curr_odom)) {
		/**
		 * I know what you're thinking... what frames are these in?
		 *
		 * curr_odom.local_frame = LOCAL_FRAME_NED
		 * curr_odom.velocity_frame = LOCAL_FRAME_FRD
		 *
		 * TODO: I still don't really know the diff between LOCAL_FRAME_NED & LOCAL_FRAME_FRD
		 *
		 * North in Gazebo is.... GREEN
		 * East in Gazebo is..... RED
		*/

		//LOCAL_FRAME_NED: In NED coordinates in earth-fixed frame
		matrix::Vector3d position({(double)curr_odom.position[0], (double)curr_odom.position[1], (double)curr_odom.position[2]});

		//Quaternion rotation from FRD body frame to earth-fixed reference frame
		matrix::Quaterniond orientation({(double)curr_odom.q[0], (double)curr_odom.q[1], (double)curr_odom.q[2], (double)curr_odom.q[3]});

		// velocity of the body frame origin in local NED earth frame
		matrix::Vector3d velocity({(double)curr_odom.velocity[0], (double)curr_odom.velocity[1], (double)curr_odom.velocity[2]});

		// In body-fixed frame (FRD) and in rad/s
		matrix::Vector3d angular_velocity{(double)curr_odom.angular_velocity[0], (double)curr_odom.angular_velocity[1], (double)curr_odom.angular_velocity[2]};

		DrewOdometry odom = {position, orientation, velocity, angular_velocity};
		_geometric_control.SetOdometry(odom);
	}

	_vehicle_status_sub.update(&_vehicle_status);

	// Perform an update for the geometric control...
	matrix::Vector3d acceleration_body_NED;
	matrix::Vector3d torque_vector_des_NED;
	_geometric_control.ComputeDesiredBodyAcceleration(&acceleration_body_NED);
	_geometric_control.ComputeDesiredTorque(&torque_vector_des_NED);

	matrix::Vector3d force_vector_des_body_FRD = _geometric_control.vehicle_mass * acceleration_body_NED; // IN PX4 NED frame

	// L1 Adaptive Augmentation - augment force and moment with adaptive terms
	matrix::Vector3d adaptive_force_NED, adaptive_moment_NED;
	DrewOdometry current_odom = _geometric_control.GetOdometry();
	_l1_adaptive.ComputeAdaptiveAugmentation(
		current_odom,
		force_vector_des_body_FRD,
		torque_vector_des_NED,
		_geometric_control.vehicle_mass,
		_geometric_control.inertia_matrix,
		&adaptive_force_NED,
		&adaptive_moment_NED
	);

	// Add adaptive terms to nominal control
	force_vector_des_body_FRD -= adaptive_force_NED;
	torque_vector_des_NED -= adaptive_moment_NED;

	matrix::Dcm<double> R_PX4_NED_TO_FLU;// Converts vectorts in PX4 FRD frame to our FLU frame
	R_PX4_NED_TO_FLU.zero();
	R_PX4_NED_TO_FLU(0,0) = 1.0;;
	R_PX4_NED_TO_FLU(1,1) = -1.0;
	R_PX4_NED_TO_FLU(2,2) = -1.0;

	matrix::Vector3d force_vector_des_FLU = R_PX4_NED_TO_FLU * force_vector_des_body_FRD;
	matrix::Vector3d torque_vector_des_FLU = R_PX4_NED_TO_FLU * torque_vector_des_NED;

	// Set the servo angles
	matrix::Vector<double, 4> alphas, betas, thrusts;
	_geometric_control.computeAlphasBetasThrustsGimbalFree(force_vector_des_FLU, torque_vector_des_FLU, alphas, betas, thrusts);

	// ====== Actuator output for gz-sim =============
	actuator_motors_s actuators{};
	actuator_servos_s actuators_servos{};
	// ================================================

	// Calculates the motor commands from the desired rotor speeds.
	// Inverse of quadratic mapping: T = (-7.41 + sqrt(7.41^2 - 4*28.2*(-0.55 - u))) / (2*28.2)
	// Original equation: u = -0.55 + 7.41*T + 28.2*T^2
	auto thrustN_to_u = [](float thrust_N) -> float {
		// T = -2.87 + 16.3u + 20.6u^2
		// Solve for u using quadratic formula: 20.6u^2 + 16.3u + (-2.87 - T) = 0
		const float a = 20.6f;
		const float b = 16.3f;
		const float c = -2.87f - thrust_N;

		const float discriminant = b * b - 4.0f * a * c;

		// Take positive root (physically meaningful solution)
		float u = (-b + sqrtf(discriminant)) / (2.0f * a);

		return fminf(1.f, fmaxf(kUidle, u));          // idle + clamp
	};

	// Sends the rotor commands to the motors
	matrix::Matrix<float, 4, 1> rotor_cmds;
	for (int i = 0; i < 4; ++i) {
		float thrust = static_cast<float>(thrusts(i));   // EXPECTS rad/s (mechanical)
		rotor_cmds(i, 0) = thrustN_to_u(thrust);
	}

	// Motors: normalized [0,1]
	actuators.control[0] = rotor_cmds(0, 0); // FL
	actuators.control[1] = rotor_cmds(1, 0); // FR
	actuators.control[2] = rotor_cmds(2, 0); // BR
	actuators.control[3] = rotor_cmds(3, 0); // BL

	// Tilt servos (alphas): normalized [-1,1] via division by pi
	actuators_servos.control[0] = (float)(alphas(0) / M_PI); // FL
	actuators_servos.control[1] = (float)(alphas(1) / M_PI); // FR
	actuators_servos.control[2] = (float)(alphas(2) / M_PI); // BR
	actuators_servos.control[3] = (float)(alphas(3) / M_PI); // BL

	// ========= Servo control for Dynamixels =======================
	// Map to actual servos
	//dynamixel_controls = {FR_b, FL_a, FR_a, FL_b, BL_a, BR_b, BR_a, BL_b }
	dynamixel_controls_s dyn_servos;
	dyn_servos.timestamp = hrt_absolute_time();

	// Alphas
	dyn_servos.dynamixel_controls[0] = (int32_t)(alphas(0)*kDynamixelConversionRate); // FL
	dyn_servos.dynamixel_controls[1] = (int32_t)(alphas(1)*kDynamixelConversionRate); // FR
	dyn_servos.dynamixel_controls[2] = (int32_t)(alphas(2)*kDynamixelConversionRate); // BR
	dyn_servos.dynamixel_controls[3] = (int32_t)(alphas(3)*kDynamixelConversionRate); // BL

	// Betas
	dyn_servos.dynamixel_controls[4] = (int32_t)(betas(0)*kDynamixelConversionRate); // FL
	dyn_servos.dynamixel_controls[5] = (int32_t)(betas(1)*kDynamixelConversionRate); // FR
	dyn_servos.dynamixel_controls[6] = (int32_t)(betas(2)*kDynamixelConversionRate); // BR
	dyn_servos.dynamixel_controls[7] = (int32_t)(betas(3)*kDynamixelConversionRate); // BL
	// ===================================================================

	// Publish all actuator messages
	actuators.timestamp = hrt_absolute_time();
	actuators_servos.timestamp = actuators.timestamp;
	actuators_servos.timestamp_sample = actuators.timestamp;

	_actuator_motors_pub.publish(actuators);
	_actuator_servos_pub.publish(actuators_servos);
	dynamixel_controls_pub.publish(dyn_servos);
} 

// Spawns a new instance of the geometric control module.
// Parses command line arguments to determine if VTOL mode should be enabled.
// `argc` - Number of command line arguments.
// `argv` - Array of command line argument strings.
// Returns PX4_OK on success, PX4_ERROR on failure.
int GeometricPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	GeometricPositionControl *instance = new GeometricPositionControl(vtol);

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

// Handles custom commands for the geometric control module.
// Currently only returns usage information for unknown commands.
// `argc` - Number of command line arguments.
// `argv` - Array of command line argument strings.
// Returns result of print_usage() call.
int GeometricPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// Prints usage information for the geometric control module.
// Displays module description and available command line options.
// `reason` - Optional reason string to display before usage information. Can be null.
// Returns 0 always.
int GeometricPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
			### Description
			This implements the multicopter rate controller. It takes rate setpoints (in acro mode
			via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

			The controller has a PID loop for angular rate error.

		)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("geometric_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int geometric_control_main(int argc, char *argv[])
{
	return ModuleBase::main(GeometricPositionControl::desc, argc, argv);
}