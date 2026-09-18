/**
 * @file bsp_current_sense.c
 * @brief 双轴电流采样链路无功率实现：注入结果读取、零偏采集和窗口校验。
 *
 * 主要接口：bsp_current_sense_init、bsp_current_sense_read_status、bsp_current_sense_read_raw、
 *          bsp_current_sense_capture_offsets、bsp_current_sense_evaluate_window。
 * 依赖关系：依赖 AT32F403A ADC 外设库、bsp_pwm.h 的时基状态、motor_current.h 的链路逻辑。
 * 关键安全约束：零偏采集前必须确认 PWM 输出禁用；本模块不改变任何引脚方向和功率门控。
 */

#include "bsp/bsp_current_sense.h"

#include "at32f403a_407.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_safe_outputs.h"
#include "motor/motor_current.h"

/** @brief 静态采样链路状态缓存，禁止动态分配。 */
static bsp_current_sense_status_t s_sense_status[AXIS_COUNT];

/** @brief 电流采样模块初始化标志。 */
static bool s_sense_initialised;

/**
 * @brief 读取当前 ADC 时钟频率。
 *
 * @return ADC 时钟频率，单位 Hz；读取失败时返回 0。
 *
 * 调用上下文：采样链路初始化与窗口评估。
 * 失败行为：APB2 频率为零时返回 0，由调用者判定窗口无效。
 */
static uint32_t clocks_refresh(void)
{
  crm_clocks_freq_type clocks;

  crm_clocks_freq_get(&clocks);
  return clocks.apb2_freq / BSP_CURRENT_SENSE_ADC_DIVIDER;
}

/**
 * @brief 返回指定轴对应的 ADC 实例。
 *
 * @param axis 轴编号。
 * @return ADC 实例指针；轴号非法时返回 ADC1。
 *
 * 调用上下文：注入结果读取。
 * 失败行为：轴号非法时由调用者前置检查排除。
 */
static adc_type *bsp_current_sense_adc(axis_id_t axis)
{
  return (axis == AXIS_B) ? ADC2 : ADC1;
}

/**
 * @brief 返回指定通道对应的注入结果枚举。
 *
 * @param channel_index 通道序号。
 * @return 注入结果枚举值。
 *
 * 调用上下文：注入结果读取。
 * 失败行为：序号非法时返回第一通道，由调用者前置检查排除。
 */
static adc_preempt_channel_type bsp_current_sense_preempt_channel(uint32_t channel_index)
{
  return (channel_index == 0u) ? ADC_PREEMPT_CHANNEL_1 : ADC_PREEMPT_CHANNEL_2;
}

/**
 * @brief 初始化电流采样模块并绑定 ADC 注入结果。
 *
 * @return true 表示初始化成功。
 *
 * 调用上下文：app_init。
 * 失败行为：ADC 未使能时返回 false，不使能任何功率输出。
 */
bool bsp_current_sense_init(void)
{
  crm_clocks_freq_type clocks;
  uint32_t adc_clock_hz = 0u;
  axis_id_t axis;

  if (s_sense_initialised) {
    return true;
  }

  /* 采样链路初始化必须建立在安全输出基础之上。 */
  bsp_safe_outputs();

  crm_clocks_freq_get(&clocks);
  adc_clock_hz = clocks.apb2_freq / BSP_CURRENT_SENSE_ADC_DIVIDER;

  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    s_sense_status[axis].axis = axis;
    s_sense_status[axis].offset_raw[0] = 0u;
    s_sense_status[axis].offset_raw[1] = 0u;
    s_sense_status[axis].last_raw[0] = 0u;
    s_sense_status[axis].last_raw[1] = 0u;
    s_sense_status[axis].offset_spread_lsb = 0u;
    s_sense_status[axis].adc_clock_hz = adc_clock_hz;
    s_sense_status[axis].conversion_ns = 0u;
    s_sense_status[axis].sample_window_ns = 0u;
    s_sense_status[axis].offsets_valid = false;
    s_sense_status[axis].window_valid = false;
    s_sense_status[axis].initialised = true;
  }

  s_sense_initialised = true;
  return true;
}

/**
 * @brief 读取指定轴的采样链路状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出状态快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：自检、日志和证据记录。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_current_sense_read_status(axis_id_t axis, bsp_current_sense_status_t *status)
{
  if (!s_sense_initialised || (status == 0) || (axis >= AXIS_COUNT)) {
    return false;
  }

  *status = s_sense_status[axis];
  return true;
}

/**
 * @brief 读取指定轴某一相电流注入结果。
 *
 * @param axis 轴编号。
 * @param channel_index 通道序号。
 * @param raw 输出原始码值。
 * @return true 表示读取成功。
 *
 * 调用上下文：控制路径和主机自检。
 * 失败行为：参数非法时返回 false，不修改输出。
 */
bool bsp_current_sense_read_raw(axis_id_t axis, uint32_t channel_index, uint16_t *raw)
{
  adc_type *adc_x;

  if (!s_sense_initialised || (raw == 0) || (axis >= AXIS_COUNT) ||
      (channel_index >= BSP_CURRENT_SENSE_CHANNEL_COUNT)) {
    return false;
  }

  adc_x = bsp_current_sense_adc(axis);
  *raw = (uint16_t)(adc_preempt_conversion_data_get(adc_x, bsp_current_sense_preempt_channel(channel_index)) &
                    (uint16_t)BSP_CURRENT_SENSE_ADC_FULL_SCALE);
  s_sense_status[axis].last_raw[channel_index] = *raw;
  return true;
}

/**
 * @brief 在零电流条件下采集两相零偏码值。
 *
 * @param axis 轴编号。
 * @return true 表示零偏采集通过离散度检查。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：输出使能或离散度过大时返回 false。
 */
bool bsp_current_sense_capture_offsets(axis_id_t axis)
{
  uint32_t sample;
  uint32_t channel;
  uint32_t sum[BSP_CURRENT_SENSE_CHANNEL_COUNT];
  uint32_t min_raw[BSP_CURRENT_SENSE_CHANNEL_COUNT];
  uint32_t max_raw[BSP_CURRENT_SENSE_CHANNEL_COUNT];
  uint32_t spread = 0u;

  if (!s_sense_initialised || (axis >= AXIS_COUNT)) {
    return false;
  }

  /* 零偏只能在功率输出禁用时采集，否则母线电流会污染结果。 */
  if (bsp_pwm_outputs_enabled(axis)) {
    s_sense_status[axis].offsets_valid = false;
    return false;
  }

  for (channel = 0u; channel < BSP_CURRENT_SENSE_CHANNEL_COUNT; ++channel) {
    sum[channel] = 0u;
    min_raw[channel] = BSP_CURRENT_SENSE_ADC_FULL_SCALE;
    max_raw[channel] = 0u;
  }

  for (sample = 0u; sample < BSP_CURRENT_SENSE_OFFSET_SAMPLES; ++sample) {
    for (channel = 0u; channel < BSP_CURRENT_SENSE_CHANNEL_COUNT; ++channel) {
      uint16_t raw = 0u;
      if (!bsp_current_sense_read_raw(axis, channel, &raw)) {
        s_sense_status[axis].offsets_valid = false;
        return false;
      }
      sum[channel] += (uint32_t)raw;
      if ((uint32_t)raw < min_raw[channel]) {
        min_raw[channel] = (uint32_t)raw;
      }
      if ((uint32_t)raw > max_raw[channel]) {
        max_raw[channel] = (uint32_t)raw;
      }
    }
  }

  for (channel = 0u; channel < BSP_CURRENT_SENSE_CHANNEL_COUNT; ++channel) {
    uint32_t channel_spread = max_raw[channel] - min_raw[channel];
    if (channel_spread > spread) {
      spread = channel_spread;
    }
    s_sense_status[axis].offset_raw[channel] =
      (uint16_t)(sum[channel] / BSP_CURRENT_SENSE_OFFSET_SAMPLES);
  }

  s_sense_status[axis].offset_spread_lsb = spread;
  s_sense_status[axis].offsets_valid = (spread <= BSP_CURRENT_SENSE_OFFSET_SPREAD_LIMIT);
  return s_sense_status[axis].offsets_valid;
}

/**
 * @brief 查询指定轴零偏是否有效。
 *
 * @param axis 轴编号。
 * @return true 表示零偏已采集且校验通过。
 *
 * 调用上下文：启动检查和故障判定。
 * 失败行为：轴号非法或未初始化时返回 false。
 */
bool bsp_current_sense_offsets_valid(axis_id_t axis)
{
  if (!s_sense_initialised || (axis >= AXIS_COUNT)) {
    return false;
  }

  return s_sense_status[axis].offsets_valid;
}

/**
 * @brief 按当前时基评估采样窗口有效性。
 *
 * @param axis 轴编号。
 * @param carrier_hz 载波频率，单位 Hz。
 * @param dead_time_ns 死区时间，单位 ns。
 * @return true 表示采样窗口不低于下限。
 *
 * 调用上下文：A4 无功率验证用例和自检。
 * 失败行为：窗口不足时返回 false 并清除窗口有效标志。
 */
bool bsp_current_sense_evaluate_window(axis_id_t axis, uint32_t carrier_hz, uint32_t dead_time_ns)
{
  uint32_t conversion_ns;
  uint32_t window_ns = 0u;

  if (!s_sense_initialised || (axis >= AXIS_COUNT)) {
    return false;
  }

  if (s_sense_status[axis].adc_clock_hz == 0u) {
    s_sense_status[axis].adc_clock_hz = clocks_refresh();
  }
  if (s_sense_status[axis].adc_clock_hz == 0u) {
    s_sense_status[axis].window_valid = false;
    return false;
  }

  if (!motor_current_conversion_ns(BSP_CURRENT_SENSE_SAMPLE_CYCLES,
                                  BSP_CURRENT_SENSE_CONVERSION_CYCLES,
                                  s_sense_status[axis].adc_clock_hz,
                                  &conversion_ns)) {
    s_sense_status[axis].window_valid = false;
    return false;
  }

  if (!motor_current_sample_window_ns(carrier_hz, dead_time_ns, conversion_ns, &window_ns)) {
    s_sense_status[axis].window_valid = false;
    return false;
  }

  s_sense_status[axis].conversion_ns = conversion_ns;
  s_sense_status[axis].sample_window_ns = window_ns;
  s_sense_status[axis].window_valid = (window_ns >= BSP_CURRENT_SENSE_MIN_WINDOW_NS);
  return s_sense_status[axis].window_valid;
}
