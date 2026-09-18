/**
 * @file motor_current.h
 * @brief 电流采样换算与量程、增益、零偏校验的纯逻辑接口。
 *
 * 主要接口：motor_current_convert、motor_current_map_phases、motor_current_validate_calibration、
 *          motor_current_conversion_ns、motor_current_sample_window_ns。
 * 依赖关系：不依赖任何外设寄存器，可在主机和 Cortex-M4F 上编译。
 * 关键安全约束：量程、增益误差或零偏误差不满足 A4 闸门时，调用者必须禁止电流环运行。
 */

#ifndef FOC_MOTOR_CURRENT_H
#define FOC_MOTOR_CURRENT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 相电流数量。 */
#define MOTOR_CURRENT_PHASE_COUNT ((uint32_t)3u)

/** @brief 参与换算的采样通道数量。 */
#define MOTOR_CURRENT_CHANNEL_COUNT ((uint32_t)2u)

/** @brief ADC 参考电压，单位 V。 */
#define MOTOR_CURRENT_ADC_VREF_V (3.3f)

/** @brief 12 位 ADC 满量程码值。 */
#define MOTOR_CURRENT_ADC_FULL_SCALE ((uint32_t)4095u)

/** @brief 电流量程下限，单位 A，对应 A4 闸门的 ±30A。 */
#define MOTOR_CURRENT_RANGE_MIN_A (30.0f)

/** @brief 增益误差门槛，按满量程比例表示，对应 A4 闸门的 3%。 */
#define MOTOR_CURRENT_GAIN_ERROR_LIMIT (0.03f)

/** @brief 零偏误差门槛，单位 V，对应 A4 闸门的 20mV。 */
#define MOTOR_CURRENT_OFFSET_ERROR_LIMIT_V (0.020f)

/** @brief 三相电流和残差相对幅值门槛。 */
#define MOTOR_CURRENT_RESIDUAL_LIMIT_RATIO (0.05f)

/** @brief 注入采样窗口下限，单位 ns。 */
#define MOTOR_CURRENT_MIN_WINDOW_NS ((uint32_t)1000u)

/** @brief 双向采样零电流工作点，单位 V，等于 ADC 参考电压的一半。 */
#define MOTOR_CURRENT_ZERO_CURRENT_V (1.65f)

/** @brief A 路候选等效增益，单位 A/V，对应 1mΩ 分流与 50 倍放大。 */
#define MOTOR_CURRENT_DEFAULT_GAIN_A_PER_V (20.0f)

/** @brief A 路候选线性量程，单位 A，对应 ±1.6V 对称摆幅。 */
#define MOTOR_CURRENT_DEFAULT_RANGE_A (32.0f)

/** @brief 电流采样标定参数，全部为静态分配。 */
typedef struct {
  uint32_t phase_map[MOTOR_CURRENT_CHANNEL_COUNT];      /**< 采样通道到电机相的映射。 */
  float polarity[MOTOR_CURRENT_CHANNEL_COUNT];          /**< 采样通道极性，取值为 +1.0 或 -1.0。 */
  float gain_a_per_v;                                   /**< 放大器加分流电阻的等效增益，单位 A/V。 */
  float offset_v[MOTOR_CURRENT_CHANNEL_COUNT];          /**< 零电流时采样通道输出电压，单位 V。 */
  float offset_error_v[MOTOR_CURRENT_CHANNEL_COUNT];    /**< 零偏标定残差，单位 V，对应 20mV 门槛。 */
  float range_a;                                        /**< 线性测量量程，单位 A。 */
  float gain_error_ratio;                               /**< 实测增益相对理论值的误差比例。 */
} motor_current_calibration_t;

/**
 * @brief 由分流电阻和放大器增益计算等效电流增益。
 *
 * @param shunt_ohm 分流电阻，单位 ohm。
 * @param amplifier_gain 放大器电压增益，无量纲。
 * @param gain_a_per_v 输出等效增益，单位 A/V。
 * @return true 表示计算成功。
 *
 * 调用上下文：硬件核对和标定构造。
 * 失败行为：空指针或任一参数非正时返回 false。
 */
bool motor_current_gain_from_hardware(float shunt_ohm, float amplifier_gain, float *gain_a_per_v);

/**
 * @brief 由等效增益和对称摆幅计算线性量程。
 *
 * @param gain_a_per_v 等效增益，单位 A/V。
 * @param half_swing_v 相对零电流工作点的对称摆幅，单位 V。
 * @param range_a 输出线性量程，单位 A。
 * @return true 表示计算成功。
 *
 * 调用上下文：硬件核对和量程门槛检查。
 * 失败行为：空指针或任一参数非正时返回 false。
 */
bool motor_current_range_from_hardware(float gain_a_per_v, float half_swing_v, float *range_a);

/**
 * @brief 计算实测增益相对理论增益的误差比例。
 *
 * @param theoretical_a_per_v 理论等效增益，单位 A/V。
 * @param measured_a_per_v 实测等效增益，单位 A/V。
 * @param error_ratio 输出误差比例，保留正负号。
 * @return true 表示计算成功。
 *
 * 调用上下文：A4 增益误差门槛检查。
 * 失败行为：空指针或任一参数非正时返回 false。
 */
bool motor_current_gain_error(float theoretical_a_per_v,
                              float measured_a_per_v,
                              float *error_ratio);

/**
 * @brief 按实板硬件参数构造标定。
 *
 * @param shunt_ohm 分流电阻，单位 ohm。
 * @param amplifier_gain 放大器电压增益，无量纲。
 * @param zero_current_v 零电流时采样通道输出电压，单位 V。
 * @param polarity 第二路采样通道极性，取值为 +1.0 或 -1.0。
 * @param range_a 线性量程，单位 A。
 * @param calibration 输出标定参数。
 * @return true 表示构造成功。
 *
 * 调用上下文：硬件核对和 A4 标定记录。
 * 失败行为：参数非法时返回 false。
 */
bool motor_current_make_calibration(float shunt_ohm,
                                    float amplifier_gain,
                                    float zero_current_v,
                                    float polarity,
                                    float range_a,
                                    motor_current_calibration_t *calibration);

/**
 * @brief 生成默认标定参数，用于目标板无功率自检。
 *
 * @param calibration 输出标定参数。
 * @return true 表示填充成功。
 *
 * 调用上下文：app_init 和主机测试。
 * 失败行为：空指针时返回 false。
 */
bool motor_current_default_calibration(motor_current_calibration_t *calibration);

/**
 * @brief 校验标定参数是否满足 A4 量程、增益和零偏门槛。
 *
 * @param calibration 标定参数。
 * @param gain_error_ratio 输出增益误差比例绝对值。
 * @param offset_error_mv 输出两通道零偏偏差绝对值，单位 mV。
 * @return true 表示全部门槛通过。
 *
 * 调用上下文：启动检查和 A4 证据记录。
 * 失败行为：参数为空或任一门槛不通过时返回 false。
 */
bool motor_current_validate_calibration(const motor_current_calibration_t *calibration,
                                       float *gain_error_ratio,
                                       uint32_t *offset_error_mv);

/**
 * @brief 把单通道 ADC 码值换算为安培值。
 *
 * @param calibration 标定参数。
 * @param raw ADC 码值。
 * @param channel_index 采样通道序号。
 * @param current_a 输出电流值，单位 A。
 * @return true 表示换算成功。
 *
 * 调用上下文：控制路径、自检和主机测试。
 * 失败行为：空指针或序号非法时返回 false，不修改输出。
 */
bool motor_current_convert(const motor_current_calibration_t *calibration,
                           uint16_t raw,
                           uint32_t channel_index,
                           float *current_a);

/**
 * @brief 按相序映射把两路采样值换算为三相电流。
 *
 * @param calibration 标定参数。
 * @param sampled 两路采样电流，单位 A。
 * @param phase_currents 输出三相电流，单位 A。
 * @return true 表示换算成功。
 *
 * 调用上下文：电流环前置处理和无功率验证。
 * 失败行为：空指针或相序映射冲突时返回 false。
 */
bool motor_current_map_phases(const motor_current_calibration_t *calibration,
                              const float *sampled,
                              float *phase_currents);

/**
 * @brief 计算三相电流和的残差及其允许上限。
 *
 * @param phase_currents 三相电流，单位 A。
 * @param residual_a 输出三相电流之和，单位 A。
 * @param limit_a 输出允许上限，单位 A。
 * @return true 表示计算成功。
 *
 * 调用上下文：采样链路一致性检查。
 * 失败行为：空指针时返回 false。
 */
bool motor_current_phase_residual(const float *phase_currents, float *residual_a, float *limit_a);

/**
 * @brief 判定三相电流残差是否在允许范围内。
 *
 * @param residual_a 三相电流之和，单位 A。
 * @param limit_a 允许上限，单位 A。
 * @return true 表示残差在范围内。
 *
 * 调用上下文：采样链路一致性检查。
 * 失败行为：不返回错误，仅返回判定结果。
 */
bool motor_current_phase_residual_is_valid(float residual_a, float limit_a);

/**
 * @brief 计算两路零偏码值的偏差，单位 mV。
 *
 * @param raw_a 第一路零偏码值。
 * @param raw_b 第二路零偏码值。
 * @return 两路零偏偏差，单位 mV。
 *
 * 调用上下文：零偏一致性检查。
 * 失败行为：无失败路径。
 */
uint32_t motor_current_offset_error_mv(uint16_t raw_a, uint16_t raw_b);

/**
 * @brief 计算零偏误差门槛对应的码值数量。
 *
 * @return 20mV 对应的 ADC 码值数量。
 *
 * 调用上下文：零偏一致性检查。
 * 失败行为：无失败路径。
 */
uint32_t motor_current_offset_limit_lsb(void);

/**
 * @brief 计算 ADC 注入转换总时间。
 *
 * @param sample_cycles 采样保持周期数。
 * @param conversion_cycles 转换周期数。
 * @param adc_clock_hz ADC 时钟，单位 Hz。
 * @param conversion_ns 输出转换总时间，单位 ns。
 * @return true 表示计算成功。
 *
 * 调用上下文：采样窗口评估。
 * 失败行为：时钟为零或空指针时返回 false。
 */
bool motor_current_conversion_ns(uint32_t sample_cycles,
                                uint32_t conversion_cycles,
                                uint32_t adc_clock_hz,
                                uint32_t *conversion_ns);

/**
 * @brief 估算中心对齐 PWM 下可用于注入采样的窗口长度。
 *
 * @param carrier_hz 载波频率，单位 Hz。
 * @param dead_time_ns 死区时间，单位 ns。
 * @param conversion_ns 注入转换总时间，单位 ns。
 * @param window_ns 输出采样窗口，单位 ns。
 * @return true 表示计算成功。
 *
 * 调用上下文：A4 采样窗口有效性检查。
 * 失败行为：参数非法或半周期不足以容纳死区与转换时返回 false。
 */
bool motor_current_sample_window_ns(uint32_t carrier_hz,
                                   uint32_t dead_time_ns,
                                   uint32_t conversion_ns,
                                   uint32_t *window_ns);

/**
 * @brief 判定采样窗口是否满足下限。
 *
 * @param window_ns 采样窗口，单位 ns。
 * @return true 表示窗口不低于下限。
 *
 * 调用上下文：A4 采样窗口有效性检查。
 * 失败行为：无失败路径。
 */
bool motor_current_sample_window_is_valid(uint32_t window_ns);

#ifdef __cplusplus
}
#endif

#endif
