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

// PREVIOUS IMPLIMENTATION
// #pragma once

// #include <px4_platform_common/module.h>
// #include <px4_platform_common/module_params.h>
// #include <uORB/SubscriptionInterval.hpp>
// #include <uORB/Publication.hpp>
// #include <uORB/PublicationMulti.hpp>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_thrust_setpoint.h>
// #include <uORB/topics/vehicle_torque_setpoint.h>
// #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


// using namespace time_literals;

// class GeometricPositionControl : public ModuleBase<GeometricPositionControl>, public ModuleParams, public px4::WorkItem
// {
// public:
// 	GeometricPositionControl(int example_param, bool example_flag);
// 	virtual ~GeometricPositionControl() = default;

// 	/** @see ModuleBase */
// 	static int task_spawn(int argc, char *argv[]);

// 	/** @see ModuleBase */
// 	static int custom_command(int argc, char *argv[]);

// 	/** @see ModuleBase */
// 	static int print_usage(const char *reason = nullptr);

// 	static GeometricPositionControl* instantiate(int argc, char *argv[]);

// 	bool init();

// private:

// 	void Run() override;

// 	/**
// 	 * Check for parameter changes and update them if needed.
// 	 * @param parameter_update_sub uorb subscription to parameter_update
// 	 * @param force for a parameter update
// 	 */
// 	void parameters_update(bool force = false);
// 	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);


// 	DEFINE_PARAMETERS(
// 		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
// 		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
// 	)

// 	// Subscriptions
// 	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

// 	// Publications
// 	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};


// };

#pragma once

#include <rate_control.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/dynamixel_controls.h>


#include <lee_position_controller.hpp>
#include <L1AdaptiveController/l1_adaptive_controller.hpp>

/*** CUSTOM ***/
#include <uORB/topics/tilting_servo_sp.h>
/*** END-CUSTOM ***/

using namespace time_literals;

extern "C" __EXPORT int geometric_control_main(int argc, char *argv[]);

class GeometricPositionControl : public ModuleBase, public ModuleParams, public px4::WorkItem
{
public:
	GeometricPositionControl(bool vtol = false);
	~GeometricPositionControl() override;

	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	void publishTorqueSetpoint(const matrix::Vector3f &torque_sp, const hrt_abstime &timestamp_sample);
	void publishThrustSetpoint(const hrt_abstime &timestamp_sample);

	RateControl _rate_control; ///< class for rate control calculations
	LeePositionController _geometric_control;
	L1AdaptiveController _l1_adaptive; ///< L1 adaptive augmentation controller

	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _landing_gear_sub{ORB_ID(landing_gear)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _v_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _v_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::SubscriptionCallbackWorkItem _vehicle_local_pos_sub {this, ORB_ID(vehicle_local_position)};
	uORB::SubscriptionCallbackWorkItem _vehicle_odometry_sub {this, ORB_ID(vehicle_odometry)};

	// Mixer readiness flag
	bool _mixer_ready{false};

	// Drew: Position setpoint stuff
	uORB::Subscription _trajectory_setpoint_sub {ORB_ID(trajectory_setpoint)};
	uORB::Subscription _local_pos_setpoint_sub {ORB_ID(vehicle_local_position_setpoint)};
	vehicle_local_position_setpoint_s _local_position_setpoint {};

	// Full state setpoint subscriptions (attitude and rates)
	uORB::Subscription _attitude_setpoint_sub {ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _rates_setpoint_sub {ORB_ID(vehicle_rates_setpoint)};

	// New dynamixel servo publisher
	uORB::Publication<dynamixel_controls_s>	dynamixel_controls_pub{ORB_ID(dynamixel_controls)};
	double theta = 0;

	uORB::Publication<actuator_motors_s>		_actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s>		_actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_v_rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	orb_advert_t _mavlink_log_pub{nullptr};

	vehicle_control_mode_s		_v_control_mode{};
	vehicle_status_s		_vehicle_status{};
	

	bool _landed{true};
	bool _maybe_landed{true};

	float _battery_status_scale{0.0f};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	matrix::Vector3f _rates_sp;			/**< angular rates setpoint */


	float		_thrust_sp{0.0f};		/**< thrust setpoint */

	int counter = 0;


	hrt_abstime _last_run{0};

	int8_t _landing_gear{landing_gear_s::GEAR_DOWN};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	/*** CUSTOM ***/
	uORB::Subscription _tilting_servo_sp_sub{ORB_ID(tilting_servo_setpoint)};
	float _tilting_angle_sp{0.0f}; /**< [rad] angle setpoint for tilting servo motors */
	matrix::Vector3f _thrust_setpoint{};
	/*** END-CUSOTM ***/

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_roll_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_r_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitch_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_p_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yaw_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_y_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,

		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,			/**< scaling factor from stick to yaw rate */

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,				/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,			/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,			/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		/*** CUSTOM ***/
		(ParamFloat<px4::params::CA_SV_TL0_MINA>) _param_tilt_min_angle,
		(ParamFloat<px4::params::CA_SV_TL0_MAXA>) _param_tilt_max_angle,
		(ParamInt<px4::params::CA_AIRFRAME>)	    _param_airframe, 		/**< 11: tilting_multirotors */

		// Geometric controller gains
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,				/**< position gain X,Y */
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,				/**< position gain Z */
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p,		/**< velocity gain X,Y */
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>) _param_mpc_z_vel_p,			/**< velocity gain Z */
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_i,				/**< position gain X,Y */
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>) _param_mpc_z_i,				/**< position gain Z */
		(ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p,				/**< attitude gain roll */
		(ParamFloat<px4::params::MC_PITCH_P>) _param_mc_pitch_p,			/**< attitude gain pitch */
		(ParamFloat<px4::params::MC_YAW_P>) _param_mc_yaw_p,				/**< attitude gain yaw */

		// L1 Adaptive Controller parameters
		(ParamInt<px4::params::L1AD_ACTIVE>) _param_l1_enable,				/**< L1 master enable/disable */
		(ParamInt<px4::params::L1AD_FORCE_EN>) _param_l1_enable_force,			/**< L1 force channel enable */
		(ParamInt<px4::params::L1AD_MOMENT_EN>) _param_l1_enable_moment,		/**< L1 moment channel enable */
		(ParamFloat<px4::params::L1AD_VEL_AS>) _param_l1_as_v,				/**< L1 velocity predictor pole */
		(ParamFloat<px4::params::L1AD_ANG_AS>) _param_l1_as_omega,			/**< L1 angular velocity predictor pole */
		(ParamFloat<px4::params::L1AD_FRC_FREQ>) _param_l1_cutoff_f,			/**< L1 force filter cutoff */
		(ParamFloat<px4::params::L1AD_MOM_FQ1>) _param_l1_cutoff_m1,			/**< L1 moment filter 1 cutoff */
		(ParamFloat<px4::params::L1AD_MOM_FQ2>) _param_l1_cutoff_m2			/**< L1 moment filter 2 cutoff */
		/*** END-CUSTOM ***/
	)

	matrix::Vector3f _acro_rate_max;	/**< max attitude rates in acro mode */

};
