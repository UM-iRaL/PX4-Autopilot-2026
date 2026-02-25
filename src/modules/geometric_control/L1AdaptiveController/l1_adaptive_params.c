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
 * @file l1_adaptive_params.c
 *
 * Parameters for L1 Adaptive Controller
 *
 * @author Jose (IRAL Lab)
 */

/**
 * L1 Adaptive Control Enable
 *
 * Master enable/disable for L1 adaptive augmentation.
 * Must be enabled for individual force/moment channels to work.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1AD_ACTIVE, 0);

/**
 * L1 Adaptive Force Channel Enable
 *
 * Enable or disable adaptive force augmentation.
 * Requires L1AD_ACTIVE to be enabled.
 * Can be disabled during takeoff to avoid interfering with acceleration ramp-up.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1AD_FORCE_EN, 0);

/**
 * L1 Adaptive Moment Channel Enable
 *
 * Enable or disable adaptive moment augmentation.
 * Requires L1AD_ACTIVE to be enabled.
 * Can be enabled during takeoff for improved attitude control.
 *
 * @boolean
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_INT32(L1AD_MOMENT_EN, 0);

/**
 * L1 State Predictor Pole for Velocity
 *
 * Negative pole location for velocity state predictor.
 * More negative values lead to faster adaptation but may cause instability.
 * Typical range: -10.0 to -1.0
 *
 * @unit 1/s
 * @min -20.0
 * @max -0.5
 * @decimal 1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1AD_VEL_AS, -10.0f);

/**
 * L1 State Predictor Pole for Angular Velocity
 *
 * Negative pole location for angular velocity state predictor.
 * More negative values lead to faster adaptation but may cause instability.
 * Typical range: -10.0 to -1.0
 *
 * @unit 1/s
 * @min -20.0
 * @max -0.5
 * @decimal 1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1AD_ANG_AS, -10.0f);

/**
 * L1 Low-Pass Filter Cutoff for Force
 *
 * Cutoff frequency for the low-pass filter on the force channel.
 * Higher values allow faster adaptation but may amplify noise.
 * Typical range: 20 to 100 Hz
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1AD_FRC_FREQ, 5.0f);

/**
 * L1 Low-Pass Filter 1 Cutoff for Moment
 *
 * Cutoff frequency for the first low-pass filter on the moment channel.
 * Two cascaded filters are used for moments to improve phase margin.
 * Typical range: 20 to 100 Hz
 *
 * @unit Hz
 * @min 0.0
 * @max 200.0
 * @decimal 1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1AD_MOM_FQ1, 1.5f);

/**
 * L1 Low-Pass Filter 2 Cutoff for Moment
 *
 * Cutoff frequency for the second low-pass filter on the moment channel.
 * Should be lower than L1AD_MOM_FQ1 for cascaded filtering.
 * Typical range: 10 to 50 Hz
 *
 * @unit Hz
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @group L1 Adaptive Control
 */
PARAM_DEFINE_FLOAT(L1AD_MOM_FQ2, 0.5f);
