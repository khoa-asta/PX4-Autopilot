#include "FuzzyAttitude.hpp"

#include <math.h>

#include <mathlib/math/Functions.hpp>

/*
 * [FUZZY-PID MODIFICATION]
 * Hai bảng dưới đây theo Bảng 3.1 và Bảng 3.2 của đồ án.
 * Hàng là de(t): NB -> PB; cột là e(t): NB -> PB.
 *
 * Bảng Kp/Ki:
 * NB: M   S   VS  VVS VS  S   M
 * NM: B   M   S   VS  S   M   B
 * NS: VB  B   M   S   M   B   VB
 * Z : VVB VB  B   M   B   VB  VVB
 * PS: VB  B   M   S   M   B   VB
 * PM: B   M   S   VS  S   M   B
 * PB: M   S   VS  VVS VS  S   M
 */
const unsigned char FuzzyAttitude::kRuleKpKi[FuzzyAttitude::kNumTerms][FuzzyAttitude::kNumTerms] = {
	{3, 2, 1, 0, 1, 2, 3},
	{4, 3, 2, 1, 2, 3, 4},
	{5, 4, 3, 2, 3, 4, 5},
	{6, 5, 4, 3, 4, 5, 6},
	{5, 4, 3, 2, 3, 4, 5},
	{4, 3, 2, 1, 2, 3, 4},
	{3, 2, 1, 0, 1, 2, 3}
};

/*
 * Bảng Kd:
 * NB: M   B   VB  VVB VB  B   M
 * NM: S   M   B   VB  B   M   S
 * NS: VS  S   M   B   M   S   VS
 * Z : VVS VS  S   M   S   VS  VVS
 * PS: VS  S   M   B   M   S   VS
 * PM: S   M   B   VB  B   M   S
 * PB: M   B   VB  VVB VB  B   M
 */
const unsigned char FuzzyAttitude::kRuleKd[FuzzyAttitude::kNumTerms][FuzzyAttitude::kNumTerms] = {
	{3, 4, 5, 6, 5, 4, 3},
	{2, 3, 4, 5, 4, 3, 2},
	{1, 2, 3, 4, 3, 2, 1},
	{0, 1, 2, 3, 2, 1, 0},
	{1, 2, 3, 4, 3, 2, 1},
	{2, 3, 4, 5, 4, 3, 2},
	{3, 4, 5, 6, 5, 4, 3}
};

void FuzzyAttitude::configure(const Config &config)
{
	_config = config;

	_config.kp_base = math::max(_config.kp_base, 0.f);
	_config.ki_base = math::max(_config.ki_base, 0.f);
	_config.kd_base = math::max(_config.kd_base, 0.f);

	_config.delta_kp_max = math::max(_config.delta_kp_max, 0.f);
	_config.delta_ki_max = math::max(_config.delta_ki_max, 0.f);
	_config.delta_kd_max = math::max(_config.delta_kd_max, 0.f);

	_config.error_max = math::max(_config.error_max, 1e-4f);
	_config.error_rate_max = math::max(_config.error_rate_max, 1e-4f);
	_config.integral_limit = math::max(_config.integral_limit, 0.f);
	_config.integral_zone = math::max(_config.integral_zone, 0.f);
	_config.derivative_cutoff_hz = math::max(_config.derivative_cutoff_hz, 0.f);
	_config.output_limit = math::max(_config.output_limit, 0.f);

	if (!_config.enabled) {
		reset();
	}
}

void FuzzyAttitude::reset()
{
	_integral_error = 0.f;
	_filtered_error_rate = 0.f;
	_derivative_initialized = false;

	_last_kp = _config.kp_base;
	_last_ki = _config.ki_base;
	_last_kd = _config.kd_base;
}

float FuzzyAttitude::membership(float x, int term_index)
{
	// Tâm 7 tập mờ: -1, -2/3, -1/3, 0, 1/3, 2/3, 1.
	constexpr float spacing = 1.f / 3.f;
	const float center = (static_cast<float>(term_index) - 3.f) * spacing;

	// NB và PB dùng dạng vai để tại biên bão hòa vẫn có độ thuộc bằng 1.
	if (term_index == 0) {
		if (x <= -1.f) {
			return 1.f;
		}

		if (x >= -1.f + spacing) {
			return 0.f;
		}

		return (-1.f + spacing - x) / spacing;
	}

	if (term_index == (kNumTerms - 1)) {
		if (x >= 1.f) {
			return 1.f;
		}

		if (x <= 1.f - spacing) {
			return 0.f;
		}

		return (x - (1.f - spacing)) / spacing;
	}

	const float distance = fabsf(x - center);

	if (distance >= spacing) {
		return 0.f;
	}

	return 1.f - distance / spacing;
}

void FuzzyAttitude::fuzzyInference(float e_norm, float de_norm,
				   float &kp_level, float &ki_level, float &kd_level) const
{
	float mu_e[kNumTerms]{};
	float mu_de[kNumTerms]{};

	for (int i = 0; i < kNumTerms; ++i) {
		mu_e[i] = membership(e_norm, i);
		mu_de[i] = membership(de_norm, i);
	}

	/*
	 * Các tâm đầu ra được chuẩn hóa từ Hình 3.6:
	 *   VVS, VS, S, M, B, VB, VVB
	 * = 0.10, 0.25, 0.40, 0.55, 0.70, 0.85, 1.00 của giá trị cực đại.
	 *
	 */
	constexpr float output_level[kNumTerms] = {
		0.10f, 0.25f, 0.40f, 0.55f, 0.70f, 0.85f, 1.00f
	};

	float numerator_kp = 0.f;
	float numerator_ki = 0.f;
	float numerator_kd = 0.f;
	float denominator = 0.f;

	for (int i_de = 0; i_de < kNumTerms; ++i_de) {
		if (mu_de[i_de] <= 0.f) {
			continue;
		}

		for (int i_e = 0; i_e < kNumTerms; ++i_e) {
			if (mu_e[i_e] <= 0.f) {
				continue;
			}

			// Mamdani AND: min(mu_de, mu_e).
			const float weight = math::min(mu_de[i_de], mu_e[i_e]);

			numerator_kp += weight * output_level[kRuleKpKi[i_de][i_e]];
			numerator_ki += weight * output_level[kRuleKpKi[i_de][i_e]];
			numerator_kd += weight * output_level[kRuleKd[i_de][i_e]];
			denominator += weight;
		}
	}

	if (denominator > 1e-6f) {
		// Giải mờ theo phương trình (3.3).
		kp_level = numerator_kp / denominator;
		ki_level = numerator_ki / denominator;
		kd_level = numerator_kd / denominator;

	} else {
		// Trường hợp dự phòng: mức M.
		kp_level = output_level[3];
		ki_level = output_level[3];
		kd_level = output_level[3];
	}
}

float FuzzyAttitude::update(float error, float error_rate, float dt)
{
	if (!_config.enabled || !isfinite(error) || !isfinite(error_rate) || !isfinite(dt) || dt <= 0.f) {
		return _config.kp_base * (isfinite(error) ? error : 0.f);
	}

	// Chặn dt để một mẫu trễ bất thường không làm tích phân thay đổi lớn.
	dt = math::constrain(dt, 0.0002f, 0.02f);

	const float e_norm = math::constrain(error / _config.error_max, -1.f, 1.f);
	const float de_norm = math::constrain(error_rate / _config.error_rate_max, -1.f, 1.f);

	float kp_level = 0.f;
	float ki_level = 0.f;
	float kd_level = 0.f;
	fuzzyInference(e_norm, de_norm, kp_level, ki_level, kd_level);

	const float kp = _config.kp_base + kp_level * _config.delta_kp_max;
	const float ki = _config.ki_base + ki_level * _config.delta_ki_max;
	const float kd = _config.kd_base + kd_level * _config.delta_kd_max;

	_last_kp = kp;
	_last_ki = ki;
	_last_kd = kd;

	// Lọc de để hạn chế nhiễu gyro đi vào nhánh D của vòng thái độ.
	if (!_derivative_initialized) {
		_filtered_error_rate = error_rate;
		_derivative_initialized = true;

	} else if (_config.derivative_cutoff_hz > 0.f) {
		constexpr float two_pi = 6.2831853071795864769f;
		const float alpha = expf(-two_pi * _config.derivative_cutoff_hz * dt);
		_filtered_error_rate = alpha * _filtered_error_rate + (1.f - alpha) * error_rate;

	} else {
		_filtered_error_rate = error_rate;
	}

	float integral_candidate = _integral_error;

	if (ki > 0.f && _config.integral_limit > 0.f) {
		if (fabsf(error) <= _config.integral_zone || _config.integral_zone <= 0.f) {
			integral_candidate = math::constrain(_integral_error + error * dt,
						      -_config.integral_limit, _config.integral_limit);

		} else {
			// Khi sai số lớn, giảm dần tích phân thay vì tiếp tục tích lũy wind-up.
			const float decay = expf(-2.f * dt);
			integral_candidate *= decay;
		}

	} else {
		integral_candidate = 0.f;
	}

	const float output_candidate = kp * error + ki * integral_candidate + kd * _filtered_error_rate;

	/*
	 * Điều kiện:
	 * nếu đầu ra đã bão hòa và sai số còn đẩy sâu hơn vào hướng bão hòa,
	 * không chấp nhận phần tích phân mới.
	 */
	bool accept_integral = true;

	if (_config.output_limit > 0.f) {
		if ((output_candidate > _config.output_limit && error > 0.f)
		    || (output_candidate < -_config.output_limit && error < 0.f)) {
			accept_integral = false;
		}
	}

	if (accept_integral) {
		_integral_error = integral_candidate;
	}

	float output = kp * error + ki * _integral_error + kd * _filtered_error_rate;

	if (_config.output_limit > 0.f) {
		output = math::constrain(output, -_config.output_limit, _config.output_limit);
	}

	return isfinite(output) ? output : 0.f;
}
