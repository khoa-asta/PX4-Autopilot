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
 * @file AttitudeControl.cpp
 */

#include "AttitudeControl.hpp"

#include <mathlib/math/Functions.hpp>

#include <cmath>

using namespace matrix;

void AttitudeControl::setProportionalGain(const Vector3f &proportional_gain, float yaw_weight)
{
	_proportional_gain = proportional_gain;
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	/*
	 * PX4 ORIGINAL BEHAVIOUR:
	 * yaw error is geometrically weighted below, therefore yaw P is divided by
	 * the same weight so small-angle yaw gain remains unchanged.
	 */
	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}

	updateFuzzyConfiguration();
}

void AttitudeControl::setRateLimit(const Vector3f &rate_limit)
{
	_rate_limit = rate_limit;
	updateFuzzyConfiguration();
}

void AttitudeControl::setFuzzyParameters(bool enabled,
					const Vector3f &base_ki,
					const Vector3f &base_kd,
					const Vector3f &delta_kp_max,
					const Vector3f &delta_ki_max,
					const Vector3f &delta_kd_max,
					float error_max,
					float error_rate_max,
					float integral_limit,
					float integral_zone,
					float derivative_cutoff_hz)
{
	const bool enable_changed = (_fuzzy_enabled != enabled);

	_fuzzy_enabled = enabled;
	_fuzzy_base_ki = base_ki;
	_fuzzy_base_kd = base_kd;
	_fuzzy_delta_kp_max = delta_kp_max;
	_fuzzy_delta_ki_max = delta_ki_max;
	_fuzzy_delta_kd_max = delta_kd_max;

	_fuzzy_error_max = error_max;
	_fuzzy_error_rate_max = error_rate_max;
	_fuzzy_integral_limit = integral_limit;
	_fuzzy_integral_zone = integral_zone;
	_fuzzy_derivative_cutoff_hz = derivative_cutoff_hz;

	updateFuzzyConfiguration();

	if (enable_changed) {
		resetFuzzy();
	}
}

void AttitudeControl::updateFuzzyConfiguration()
{
	FuzzyAttitude::Config config{};
	config.enabled = _fuzzy_enabled;
	config.error_max = _fuzzy_error_max;
	config.error_rate_max = _fuzzy_error_rate_max;
	config.integral_limit = _fuzzy_integral_limit;
	config.integral_zone = _fuzzy_integral_zone;
	config.derivative_cutoff_hz = _fuzzy_derivative_cutoff_hz;

	config.kp_base = _proportional_gain(0);
	config.ki_base = _fuzzy_base_ki(0);
	config.kd_base = _fuzzy_base_kd(0);
	config.delta_kp_max = _fuzzy_delta_kp_max(0);
	config.delta_ki_max = _fuzzy_delta_ki_max(0);
	config.delta_kd_max = _fuzzy_delta_kd_max(0);
	config.output_limit = _rate_limit(0);
	_fuzzy_roll.configure(config);

	config.kp_base = _proportional_gain(1);
	config.ki_base = _fuzzy_base_ki(1);
	config.kd_base = _fuzzy_base_kd(1);
	config.delta_kp_max = _fuzzy_delta_kp_max(1);
	config.delta_ki_max = _fuzzy_delta_ki_max(1);
	config.delta_kd_max = _fuzzy_delta_kd_max(1);
	config.output_limit = _rate_limit(1);
	_fuzzy_pitch.configure(config);

	/*
	 * [FUZZY-PID MODIFICATION]
	 * Bù yaw_weight cho cả P, I, D và các delta gain. 
	 */
	const float yaw_gain_compensation = (_yaw_w > 1e-4f) ? (1.f / _yaw_w) : 0.f;

	config.kp_base = (_yaw_w > 1e-4f) ? _proportional_gain(2) : 0.f;
	config.ki_base = _fuzzy_base_ki(2) * yaw_gain_compensation;
	config.kd_base = _fuzzy_base_kd(2) * yaw_gain_compensation;
	config.delta_kp_max = _fuzzy_delta_kp_max(2) * yaw_gain_compensation;
	config.delta_ki_max = _fuzzy_delta_ki_max(2) * yaw_gain_compensation;
	config.delta_kd_max = _fuzzy_delta_kd_max(2) * yaw_gain_compensation;
	config.output_limit = _rate_limit(2);
	_fuzzy_yaw.configure(config);
}

void AttitudeControl::resetFuzzy()
{
	_fuzzy_roll.reset();
	_fuzzy_pitch.reset();
	_fuzzy_yaw.reset();
}

Vector3f AttitudeControl::update(const Quatf &q, const Vector3f &angular_rates, float dt)
{
	Quatf qd = _attitude_setpoint_q;

	// ===== PX4 ORIGINAL: reduced desired attitude, prioritize roll/pitch over yaw =====
	const Vector3f e_z = q.dcm_z();
	const Vector3f e_z_d = qd.dcm_z();
	Quatf qd_red(e_z, e_z_d);

	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		qd_red = qd;

	} else {
		qd_red *= q;
	}

	Quatf qd_dyaw = qd_red.inversed() * qd;
	qd_dyaw.canonicalize();
	qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
	qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

	qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))),
			     0.f, 0.f,
			     sinf(_yaw_w * asinf(qd_dyaw(3))));

	// Quaternion error from current attitude q to desired attitude qd.
	const Quatf qe = q.inversed() * qd;
	const Vector3f eq = 2.f * qe.canonical().imag();
	// ===== END PX4 ORIGINAL GEOMETRY =====

	Vector3f rate_setpoint{};

	if (_fuzzy_enabled) {
		/*
		 * [FUZZY-PID MODIFICATION]
		 * dùng ba Fuzzy PID độc lập để tạo rate setpoint.
		 *
		 */
		Vector3f error_rate = -angular_rates;

		// qd yaw đã được nhân yaw_weight; de yaw cũng phải được nhân tương ứng.
		error_rate(2) *= _yaw_w;

		rate_setpoint(0) = _fuzzy_roll.update(eq(0), error_rate(0), dt);
		rate_setpoint(1) = _fuzzy_pitch.update(eq(1), error_rate(1), dt);
		rate_setpoint(2) = _fuzzy_yaw.update(eq(2), error_rate(2), dt);

	} else {
		// PX4 ORIGINAL fallback: giữ nguyên bộ P quaternion khi MC_FUZZY_EN = 0.
		rate_setpoint = eq.emult(_proportional_gain);
	}

	// PX4 ORIGINAL: yaw-rate feed-forward expressed in body frame.
	if (std::isfinite(_yawspeed_setpoint)) {
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	for (int i = 0; i < 3; ++i) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
