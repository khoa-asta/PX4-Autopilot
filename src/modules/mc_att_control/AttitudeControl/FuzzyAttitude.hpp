#pragma once

/**
 * @brief Bộ điều khiển PID tự chỉnh định bằng logic mờ cho một trục thái độ.
 *
 * Hai đầu vào của hệ mờ:
 *  - e: sai số thái độ lấy từ vectơ sai số quaternion.
 *  - de: tốc độ biến thiên sai số, được xấp xỉ từ tốc độ góc thân.
 *
 * Ba đầu ra của hệ mờ:
 *  - delta Kp, delta Ki, delta Kd.
 *
 * Hệ số PID tức thời:
 *  Kp = Kp0 + delta Kp
 *  Ki = Ki0 + delta Ki
 *  Kd = Kd0 + delta Kd
 *
 * Luật mờ bám theo Bảng 3.1 và Bảng 3.2 trong đồ án.
 */
class FuzzyAttitude
{
public:
	FuzzyAttitude() = default;
	~FuzzyAttitude() = default;

	/** Cấu hình đầy đủ cho một trục. */
	struct Config {
		bool enabled{false};

		float kp_base{0.f};
		float ki_base{0.f};
		float kd_base{0.f};

		float delta_kp_max{0.f};
		float delta_ki_max{0.f};
		float delta_kd_max{0.f};

		float error_max{0.5f};          // Miền e: [-error_max, error_max]
		float error_rate_max{0.8726646f}; // Miền de, mặc định 50 deg/s

		float integral_limit{0.15f};    // Giới hạn |integral(e)|
		float integral_zone{0.20f};     // Chỉ tích phân mạnh khi |e| nằm trong miền này
		float derivative_cutoff_hz{20.f};
		float output_limit{0.f};        //  không giới hạn tại lớp này
	};

	void configure(const Config &config);
	void reset();

	/**
	 * @brief Chạy một chu kỳ điều khiển.
	 * @param error Sai số thái độ theo một trục.
	 * @param error_rate Tốc độ biến thiên sai số xấp xỉ [1/s hoặc rad/s].
	 * @param dt Chu kỳ điều khiển [s].
	 * @return Tốc độ góc đặt của trục [rad/s].
	 */
	float update(float error, float error_rate, float dt);

	bool enabled() const { return _config.enabled; }

	float lastKp() const { return _last_kp; }
	float lastKi() const { return _last_ki; }
	float lastKd() const { return _last_kd; }

private:
	static constexpr int kNumTerms = 7;

	// Thứ tự biến ngôn ngữ đầu vào:
	// 0 NB, 1 NM, 2 NS, 3 Z, 4 PS, 5 PM, 6 PB.
	// Thứ tự mức đầu ra:
	// 0 VVS, 1 VS, 2 S, 3 M, 4 B, 5 VB, 6 VVB.
	static const unsigned char kRuleKpKi[kNumTerms][kNumTerms];
	static const unsigned char kRuleKd[kNumTerms][kNumTerms];

	/** Hàm liên thuộc tam giác trên miền chuẩn hóa [-1, 1]. */
	static float membership(float normalized_input, int term_index);

	/** Tính mức đầu ra chuẩn hóa của ba hệ số bằng luật AND-min và giải mờ trọng tâm. */
	void fuzzyInference(float error_normalized, float error_rate_normalized,
			    float &kp_level, float &ki_level, float &kd_level) const;

	Config _config{};

	float _integral_error{0.f};
	float _filtered_error_rate{0.f};
	bool _derivative_initialized{false};

	float _last_kp{0.f};
	float _last_ki{0.f};
	float _last_kd{0.f};
};
