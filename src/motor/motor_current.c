/**
 * @file motor_current.c
 * @brief 电流采样换算、量程校验和采样窗口计算的纯逻辑实现。
 *
 * 主要接口：motor_current_convert、motor_current_map_phases、motor_current_validate_calibration、
 *          motor_current_make_calibration、motor_current_conversion_ns、motor_current_sample_window_ns。
 * 依赖关系：只依赖 C11 标准库，可同时用于主机测试和目标固件。
 * 关键安全约束：任一门槛不通过时返回 false，禁止上层继续运行电流环。
 */

#include "motor/motor_current.h"

/**
 * @brief 计算单精度浮点绝对值。
 *
 * @param value 输入值。
 * @return 绝对值。
 *
 * 调用上下文：标定校验。
 * 失败行为：无失败路径。
 */
static float motor_current_abs_f(float value)
{
  return (value < 0.0f) ? -value : value;
}

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
bool motor_current_gain_from_hardware(float shunt_ohm, float amplifier_gain, float *gain_a_per_v)
{
  if ((gain_a_per_v == 0) || !(shunt_ohm > 0.0f) || !(amplifier_gain > 0.0f)) {
    return false;
  }

  /* 分流电阻把电流转成电压，放大器再放大该电压，因此增益为乘积的倒数。 */
  *gain_a_per_v = 1.0f / (shunt_ohm * amplifier_gain);
  return true;
}

/**
 * @brief 由等效增益和放大器对称摆幅计算线性量程。
 *
 * @param gain_a_per_v 等效增益，单位 A/V。
 * @param half_swing_v 放大器相对零电流工作点的对称摆幅，单位 V。
 * @param range_a 输出线性量程，单位 A。
 * @return true 表示计算成功。
 *
 * 调用上下文：硬件核对和量程门槛检查。
 * 失败行为：空指针或任一参数非正时返回 false。
 */
bool motor_current_range_from_hardware(float gain_a_per_v, float half_swing_v, float *range_a)
{
  if ((range_a == 0) || !(gain_a_per_v > 0.0f) || !(half_swing_v > 0.0f)) {
    return false;
  }

  *range_a = gain_a_per_v * half_swing_v;
  return true;
}

/**
 * @brief 计算实测增益相对理论增益的误差比例。
 *
 * @param theoretical_a_per_v 理论等效增益，单位 A/V。
 * @param measured_a_per_v 实测等效增益，单位 A/V。
 * @param error_ratio 输出误差比例，正负号保留方向。
 * @return true 表示计算成功。
 *
 * 调用上下文：A4 增益误差门槛检查。
 * 失败行为：空指针或任一参数非正时返回 false。
 */
bool motor_current_gain_error(float theoretical_a_per_v,
                              float measured_a_per_v,
                              float *error_ratio)
{
  if ((error_ratio == 0) || !(theoretical_a_per_v > 0.0f) || !(measured_a_per_v > 0.0f)) {
    return false;
  }

  *error_ratio = (measured_a_per_v - theoretical_a_per_v) / theoretical_a_per_v;
  return true;
}

/**
 * @brief 按实板分流、放大、零电流工作点、极性和量程构造标定参数。
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
 * 失败行为：空指针、极性非法、参数非正或增益计算失败时返回 false。
 */
bool motor_current_make_calibration(float shunt_ohm,
                                    float amplifier_gain,
                                    float zero_current_v,
                                    float polarity,
                                    float range_a,
                                    motor_current_calibration_t *calibration)
{
  float gain_a_per_v = 0.0f;
  uint32_t channel;

  if (calibration == 0) {
    return false;
  }
  if ((polarity != 1.0f) && (polarity != -1.0f)) {
    return false;
  }
  if (!(zero_current_v > 0.0f) || !(range_a > 0.0f)) {
    return false;
  }
  if (!motor_current_gain_from_hardware(shunt_ohm, amplifier_gain, &gain_a_per_v)) {
    return false;
  }

  calibration->gain_a_per_v = gain_a_per_v;
  calibration->range_a = range_a;
  calibration->gain_error_ratio = 0.0f;
  calibration->phase_map[0] = 0u;
  calibration->phase_map[1] = 1u;

  for (channel = 0u; channel < MOTOR_CURRENT_CHANNEL_COUNT; ++channel) {
    calibration->polarity[channel] = 1.0f;
    calibration->offset_v[channel] = zero_current_v;
    calibration->offset_error_v[channel] = 0.0f;
  }

  /* 极性和零电流工作点由实板接线决定，当前只作用于第二路采样通道。 */
  calibration->polarity[1] = polarity;
  return true;
}

/**
 * @brief 生成默认标定参数。
 *
 * @param calibration 输出标定参数。
 * @return true 表示填充成功。
 *
 * 调用上下文：app_init 和主机测试。
 * 失败行为：空指针时返回 false。
 */
bool motor_current_default_calibration(motor_current_calibration_t *calibration)
{
  uint32_t channel;

  if (calibration == 0) {
    return false;
  }

  /* A 路候选硬件：1mΩ 分流加 50 倍放大，零电流工作点 1.65V，对称量程 ±32A。 */
  calibration->gain_a_per_v = MOTOR_CURRENT_DEFAULT_GAIN_A_PER_V;
  calibration->range_a = MOTOR_CURRENT_DEFAULT_RANGE_A;
  calibration->gain_error_ratio = 0.0f;
  calibration->phase_map[0] = 0u;
  calibration->phase_map[1] = 1u;

  for (channel = 0u; channel < MOTOR_CURRENT_CHANNEL_COUNT; ++channel) {
    calibration->polarity[channel] = 1.0f;
    calibration->offset_v[channel] = MOTOR_CURRENT_ZERO_CURRENT_V;
    calibration->offset_error_v[channel] = 0.0f;
  }

  return true;
}

/**
 * @brief 校验标定参数是否满足 A4 门槛。
 *
 * @param calibration 标定参数。
 * @param gain_error_ratio 输出增益误差比例绝对值。
 * @param offset_error_mv 输出两通道零偏偏差绝对值，单位 mV。
 * @return true 表示全部门槛通过。
 *
 * 调用上下文：启动检查和 A4 证据记录。
 * 失败行为：任一门槛不通过时返回 false。
 */
bool motor_current_validate_calibration(const motor_current_calibration_t *calibration,
                                       float *gain_error_ratio,
                                       uint32_t *offset_error_mv)
{
  uint32_t channel;
  uint32_t max_offset_mv = 0u;
  float offset_mv;

  if ((calibration == 0) || (gain_error_ratio == 0) || (offset_error_mv == 0)) {
    return false;
  }

  *gain_error_ratio = motor_current_abs_f(calibration->gain_error_ratio);
  *offset_error_mv = 0u;

  if (motor_current_abs_f(calibration->range_a) < MOTOR_CURRENT_RANGE_MIN_A) {
    return false;
  }
  if (calibration->gain_a_per_v <= 0.0f) {
    return false;
  }
  if (*gain_error_ratio > MOTOR_CURRENT_GAIN_ERROR_LIMIT) {
    return false;
  }

  for (channel = 0u; channel < MOTOR_CURRENT_CHANNEL_COUNT; ++channel) {
    if ((calibration->polarity[channel] != 1.0f) && (calibration->polarity[channel] != -1.0f)) {
      return false;
    }
    offset_mv = motor_current_abs_f(calibration->offset_error_v[channel]) * 1000.0f;
    if (offset_mv > (MOTOR_CURRENT_OFFSET_ERROR_LIMIT_V * 1000.0f)) {
      return false;
    }
    if ((uint32_t)offset_mv > max_offset_mv) {
      max_offset_mv = (uint32_t)offset_mv;
    }
  }

  *offset_error_mv = max_offset_mv;
  return true;
}

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
 * 失败行为：空指针或序号非法时返回 false。
 */
bool motor_current_convert(const motor_current_calibration_t *calibration,
                           uint16_t raw,
                           uint32_t channel_index,
                           float *current_a)
{
  float volts;

  if ((calibration == 0) || (current_a == 0) || (channel_index >= MOTOR_CURRENT_CHANNEL_COUNT)) {
    return false;
  }

  volts = ((float)raw * MOTOR_CURRENT_ADC_VREF_V) / (float)MOTOR_CURRENT_ADC_FULL_SCALE;
  volts -= calibration->offset_v[channel_index];
  volts *= calibration->polarity[channel_index];
  *current_a = volts * calibration->gain_a_per_v;
  return true;
}

/**
 * @brief 按相序映射把两路采样值换算为三相电流。
 *
 * @param calibration 标定参数。
 * @param sampled 两路采样电流，单位 A。
 * @param phase_currents 输出三相电流，单位 A。
 * @return true 表示换算成功。
 *
 * 调用上下文：电流环前置处理和无功率验证。
 * 失败行为：相序映射冲突时返回 false。
 */
bool motor_current_map_phases(const motor_current_calibration_t *calibration,
                              const float *sampled,
                              float *phase_currents)
{
  uint32_t first;
  uint32_t second;
  uint32_t rest;

  if ((calibration == 0) || (sampled == 0) || (phase_currents == 0)) {
    return false;
  }

  first = calibration->phase_map[0];
  second = calibration->phase_map[1];
  if ((first >= MOTOR_CURRENT_PHASE_COUNT) || (second >= MOTOR_CURRENT_PHASE_COUNT) ||
      (first == second)) {
    return false;
  }

  phase_currents[first] = sampled[0];
  phase_currents[second] = sampled[1];

  /* 三相无中线时第三相由基尔霍夫电流定律直接得到。 */
  rest = (MOTOR_CURRENT_PHASE_COUNT - first) - second;
  phase_currents[rest] = -(sampled[0] + sampled[1]);
  return true;
}

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
bool motor_current_phase_residual(const float *phase_currents, float *residual_a, float *limit_a)
{
  uint32_t phase;
  float magnitude = 0.0f;

  if ((phase_currents == 0) || (residual_a == 0) || (limit_a == 0)) {
    return false;
  }

  *residual_a = phase_currents[0] + phase_currents[1] + phase_currents[2];
  for (phase = 0u; phase < MOTOR_CURRENT_PHASE_COUNT; ++phase) {
    float abs_value = motor_current_abs_f(phase_currents[phase]);
    if (abs_value > magnitude) {
      magnitude = abs_value;
    }
  }

  *limit_a = magnitude * MOTOR_CURRENT_RESIDUAL_LIMIT_RATIO;
  return true;
}

/**
 * @brief 判定三相电流残差是否在允许范围内。
 *
 * @param residual_a 三相电流之和，单位 A。
 * @param limit_a 允许上限，单位 A。
 * @return true 表示残差在范围内。
 *
 * 调用上下文：采样链路一致性检查。
 * 失败行为：无失败路径。
 */
bool motor_current_phase_residual_is_valid(float residual_a, float limit_a)
{
  return motor_current_abs_f(residual_a) <= motor_current_abs_f(limit_a);
}

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
uint32_t motor_current_offset_error_mv(uint16_t raw_a, uint16_t raw_b)
{
  uint32_t delta = (raw_a > raw_b) ? ((uint32_t)raw_a - (uint32_t)raw_b)
                                   : ((uint32_t)raw_b - (uint32_t)raw_a);
  uint32_t millivolts = (delta * 3300u) / MOTOR_CURRENT_ADC_FULL_SCALE;

  return millivolts;
}

/**
 * @brief 计算零偏误差门槛对应的码值数量。
 *
 * @return 20mV 对应的 ADC 码值数量。
 *
 * 调用上下文：零偏一致性检查。
 * 失败行为：无失败路径。
 */
uint32_t motor_current_offset_limit_lsb(void)
{
  uint32_t limit_lsb =
    (uint32_t)((MOTOR_CURRENT_OFFSET_ERROR_LIMIT_V * 1000.0f * (float)MOTOR_CURRENT_ADC_FULL_SCALE) /
               (MOTOR_CURRENT_ADC_VREF_V * 1000.0f));

  return (limit_lsb == 0u) ? 1u : limit_lsb;
}

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
                                uint32_t *conversion_ns)
{
  uint64_t total_cycles;

  if ((conversion_ns == 0) || (adc_clock_hz == 0u)) {
    return false;
  }

  total_cycles = (uint64_t)sample_cycles + (uint64_t)conversion_cycles;
  if (total_cycles == 0u) {
    return false;
  }

  *conversion_ns =
    (uint32_t)(((total_cycles * UINT64_C(1000000000)) + (uint64_t)adc_clock_hz - 1u) /
               (uint64_t)adc_clock_hz);
  return true;
}

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
 * 失败行为：半周期不足以容纳死区和转换时间时返回 false。
 */
bool motor_current_sample_window_ns(uint32_t carrier_hz,
                                   uint32_t dead_time_ns,
                                   uint32_t conversion_ns,
                                   uint32_t *window_ns)
{
  uint64_t half_period_ns;
  uint64_t occupied_ns;

  if ((window_ns == 0) || (carrier_hz == 0u)) {
    return false;
  }

  /* 中心对齐模式下每个载波周期有两次对称采样机会，窗口按半周期估算。 */
  half_period_ns = UINT64_C(500000000) / (uint64_t)carrier_hz;
  occupied_ns = (uint64_t)dead_time_ns + (uint64_t)conversion_ns;
  if (half_period_ns <= occupied_ns) {
    *window_ns = 0u;
    return false;
  }

  *window_ns = (uint32_t)(half_period_ns - occupied_ns);
  return true;
}

/**
 * @brief 判定采样窗口是否满足下限。
 *
 * @param window_ns 采样窗口，单位 ns。
 * @return true 表示窗口不低于下限。
 *
 * 调用上下文：A4 采样窗口有效性检查。
 * 失败行为：无失败路径。
 */
bool motor_current_sample_window_is_valid(uint32_t window_ns)
{
  return window_ns >= MOTOR_CURRENT_MIN_WINDOW_NS;
}
