/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.hpp
 *
 * A quaternion based attitude controller.
 *
 * @author Matthias Grob	<maetugr@gmail.com>
 *
 * Publication documenting the implemented Quaternion Attitude Control:
 * Nonlinear Quadrocopter Attitude Control (2013)
 * by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 * Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 * https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 */

/**
 * @file AttitudeControl.hpp
 *
 * Quaternion attitude controller extended with a Fuzzy self-tuning PID outer loop.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

#include "FuzzyAttitude.hpp"

class AttitudeControl
{
public:
	AttitudeControl() = default;
	~AttitudeControl() = default;

	/**
	 * Set base proportional attitude gains.
	 * These remain the original PX4 MC_ROLL_P, MC_PITCH_P and MC_YAW_P gains.
	 */
	void setProportionalGain(const matrix::Vector3f &proportional_gain, float yaw_weight);

	/** Set hard limit for output angular-rate setpoints [rad/s]. */
	void setRateLimit(const matrix::Vector3f &rate_limit);

	/**
	 * [FUZZY-PID MODIFICATION]
	 * Configure the Fuzzy PID increments and the new I/D base gains.
	 *
	 * PX4 gốc chỉ có P tại vòng thái độ. Đồ án dùng công thức
	 * (Kp + dKp)e + (Ki + dKi)Integral(e) + (Kd + dKd)de.
	 */
	void setFuzzyParameters(bool enabled,
				const matrix::Vector3f &base_ki,
				const matrix::Vector3f &base_kd,
				const matrix::Vector3f &delta_kp_max,
				const matrix::Vector3f &delta_ki_max,
				const matrix::Vector3f &delta_kd_max,
				float error_max,
				float error_rate_max,
				float integral_limit,
				float integral_zone,
				float derivative_cutoff_hz);

	void resetFuzzy();
	bool fuzzyEnabled() const { return _fuzzy_enabled; }

	void setAttitudeSetpoint(const matrix::Quatf &qd, float yawspeed_setpoint)
	{
		_attitude_setpoint_q = qd;
		_attitude_setpoint_q.normalize();
		_yawspeed_setpoint = yawspeed_setpoint;
	}

	void adaptAttitudeSetpoint(const matrix::Quatf &q_delta)
	{
		_attitude_setpoint_q = q_delta * _attitude_setpoint_q;
		_attitude_setpoint_q.normalize();
	}

	/**
	 * Run one control cycle.
	 * @param q Current attitude quaternion.
	 * @param angular_rates Current body angular rates [rad/s].
	 * @param dt Control period [s].
	 * @return Body angular-rate setpoint [rad/s].
	 */
	matrix::Vector3f update(const matrix::Quatf &q,
				const matrix::Vector3f &angular_rates,
				float dt);

private:
	void updateFuzzyConfiguration();

	matrix::Vector3f _proportional_gain{};
	matrix::Vector3f _rate_limit{};
	float _yaw_w{0.f};

	matrix::Quatf _attitude_setpoint_q{};
	float _yawspeed_setpoint{0.f};

	// FUZZY-PID
	FuzzyAttitude _fuzzy_roll;
	FuzzyAttitude _fuzzy_pitch;
	FuzzyAttitude _fuzzy_yaw;

	bool _fuzzy_enabled{false};
	matrix::Vector3f _fuzzy_base_ki{};
	matrix::Vector3f _fuzzy_base_kd{};
	matrix::Vector3f _fuzzy_delta_kp_max{};
	matrix::Vector3f _fuzzy_delta_ki_max{};
	matrix::Vector3f _fuzzy_delta_kd_max{};

	float _fuzzy_error_max{0.5f};
	float _fuzzy_error_rate_max{0.8726646f};
	float _fuzzy_integral_limit{0.15f};
	float _fuzzy_integral_zone{0.20f};
	float _fuzzy_derivative_cutoff_hz{20.f};
};
