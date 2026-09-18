/**
 * @file bsp_pwm_math.c
 * @brief PWM 时基、死区和最小脉宽的纯计算实现。
 *
 * 主要接口：bsp_pwm_timer_clock_hz、bsp_pwm_compute_timing、bsp_pwm_encode_dead_time、
 *          bsp_pwm_dead_time_ns、bsp_pwm_min_pulse_ticks、bsp_pwm_timing_is_within_limits。
 * 依赖关系：只依赖 bsp_pwm.h 的标准类型定义，不依赖任何厂商寄存器。
 * 关键安全约束：本文件不访问硬件，所有结果必须由 bsp_pwm.c 写入寄存器后回读确认。
 */

#include "bsp/bsp_pwm.h"

/** @brief 死区编码的段数和段步长，单位 t_dts。 */
#define BSP_PWM_DEAD_TIME_SEGMENTS ((uint32_t)4u)

/** @brief 每秒纳秒数，用于整数换算。 */
#define BSP_PWM_NANOSECONDS_PER_SECOND ((uint64_t)1000000000u)

/**
 * @brief 计算向上取整的整数除法。
 *
 * @param numerator 被除数。
 * @param denominator 除数，必须大于零。
 * @return 向上取整的商。
 *
 * 调用上下文：纯计算内部。
 * 失败行为：除数为零时返回 0，由调用者前置检查排除。
 */
static uint64_t bsp_pwm_div_ceil(uint64_t numerator, uint64_t denominator)
{
  if (denominator == 0u) {
    return 0u;
  }
  return (numerator + denominator - 1u) / denominator;
}

/**
 * @brief 把 t_dts 个数换算为纳秒。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param dts_count t_dts 个数。
 * @return 纳秒时间。
 *
 * 调用上下文：死区编码和解码。
 * 失败行为：时钟为零时返回 0。
 */
static uint32_t bsp_pwm_dts_to_ns(uint32_t timer_clock_hz, uint32_t dts_count)
{
  if (timer_clock_hz == 0u) {
    return 0u;
  }
  return (uint32_t)(((uint64_t)dts_count * BSP_PWM_NANOSECONDS_PER_SECOND) / (uint64_t)timer_clock_hz);
}

/**
 * @brief 按 APB 与 AHB 频率计算定时器计数时钟。
 *
 * @param ahb_freq_hz AHB 频率，单位 Hz。
 * @param apb_freq_hz APB 频率，单位 Hz。
 * @param timer_clock_hz 输出定时器计数时钟，单位 Hz。
 * @return true 表示计算成功。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：参数非法时返回 false。
 */
bool bsp_pwm_timer_clock_hz(uint32_t ahb_freq_hz, uint32_t apb_freq_hz, uint32_t *timer_clock_hz)
{
  if ((timer_clock_hz == 0) || (ahb_freq_hz == 0u) || (apb_freq_hz == 0u)) {
    return false;
  }

  /* APB 分频为 1 时定时器时钟等于 APB 时钟，其他情况按两倍 APB 计算。 */
  if (apb_freq_hz == ahb_freq_hz) {
    *timer_clock_hz = apb_freq_hz;
  } else {
    *timer_clock_hz = apb_freq_hz * 2u;
  }

  return (*timer_clock_hz != 0u);
}

/**
 * @brief 计算中心对齐时基的预分频和自动重装值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param carrier_hz 目标载波频率，单位 Hz。
 * @param prescaler 输出预分频值。
 * @param period_ticks 输出自动重装值。
 * @param actual_carrier_hz 输出实际载波频率，单位 Hz。
 * @return true 表示找到满足分辨率要求的时基组合。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：无法在计数器范围内达到目标频率时返回 false。
 */
bool bsp_pwm_compute_timing(uint32_t timer_clock_hz,
                            uint32_t carrier_hz,
                            uint32_t *prescaler,
                            uint32_t *period_ticks,
                            uint32_t *actual_carrier_hz)
{
  uint32_t divider;

  if ((timer_clock_hz == 0u) || (carrier_hz == 0u) ||
      (prescaler == 0) || (period_ticks == 0) || (actual_carrier_hz == 0)) {
    return false;
  }

  for (divider = 1u; divider <= (BSP_PWM_PRESCALER_MAX + 1u); ++divider) {
    uint32_t counter_clock = timer_clock_hz / divider;
    uint32_t period;

    if (counter_clock < (carrier_hz * 2u)) {
      break;
    }

    /* 中心对齐时一个载波周期包含两次计数扫描。 */
    period = counter_clock / (carrier_hz * 2u);
    if ((period == 0u) || (period > BSP_PWM_PERIOD_MAX)) {
      continue;
    }

    *prescaler = divider - 1u;
    *period_ticks = period;
    *actual_carrier_hz = counter_clock / (period * 2u);
    return (*actual_carrier_hz != 0u);
  }

  return false;
}

/**
 * @brief 把死区时间编码为定时器死区发生器取值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param dead_time_ns 目标死区时间，单位 ns。
 * @param encoded 输出死区编码值。
 * @param actual_ns 输出实际死区时间，单位 ns。
 * @return true 表示编码落在定时器支持范围内。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：请求值超过最大可编码死区时返回 false。
 */
bool bsp_pwm_encode_dead_time(uint32_t timer_clock_hz,
                              uint32_t dead_time_ns,
                              uint32_t *encoded,
                              uint32_t *actual_ns)
{
  static const uint32_t segment_base[BSP_PWM_DEAD_TIME_SEGMENTS] = {0u, 128u, 256u, 512u};
  static const uint32_t segment_step[BSP_PWM_DEAD_TIME_SEGMENTS] = {1u, 2u, 8u, 16u};
  static const uint32_t segment_max[BSP_PWM_DEAD_TIME_SEGMENTS] = {127u, 63u, 31u, 31u};
  uint64_t required_dts;
  uint32_t best_code = 0u;
  uint32_t best_dts = 0u;
  bool found = false;
  uint32_t index;

  if ((timer_clock_hz == 0u) || (encoded == 0) || (actual_ns == 0)) {
    return false;
  }

  if (dead_time_ns == 0u) {
    *encoded = 0u;
    *actual_ns = 0u;
    return true;
  }

  /* 需要的 t_dts 个数按向上取整计算，保证实际死区不小于请求值。 */
  required_dts = bsp_pwm_div_ceil((uint64_t)dead_time_ns * (uint64_t)timer_clock_hz,
                                  BSP_PWM_NANOSECONDS_PER_SECOND);
  if (required_dts == 0u) {
    required_dts = 1u;
  }

  for (index = 0u; index < BSP_PWM_DEAD_TIME_SEGMENTS; ++index) {
    uint32_t step = segment_step[index];
    uint32_t base = segment_base[index];
    uint32_t max_dts = base + (step * segment_max[index]);
    uint32_t candidate_dts;
    uint32_t candidate_code;

    if ((uint64_t)max_dts < required_dts) {
      continue;
    }

    if (required_dts <= (uint64_t)base) {
      candidate_dts = base;
    } else {
      uint64_t delta = bsp_pwm_div_ceil(required_dts - (uint64_t)base, (uint64_t)step);
      if (delta > (uint64_t)segment_max[index]) {
        continue;
      }
      candidate_dts = base + ((uint32_t)delta * step);
    }

    if (index == 0u) {
      candidate_code = candidate_dts;
    } else if (index == 1u) {
      candidate_code = 0x80u | ((candidate_dts - base) / step);
    } else if (index == 2u) {
      candidate_code = 0xC0u | ((candidate_dts - base) / step);
    } else {
      candidate_code = 0xE0u | ((candidate_dts - base) / step);
    }

    if (!found || (candidate_dts < best_dts)) {
      best_dts = candidate_dts;
      best_code = candidate_code;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  *encoded = best_code;
  *actual_ns = bsp_pwm_dts_to_ns(timer_clock_hz, best_dts);
  return (*actual_ns >= dead_time_ns);
}

/**
 * @brief 把死区编码值反算为时间。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param encoded 死区编码值。
 * @param actual_ns 输出实际死区时间，单位 ns。
 * @return true 表示反算成功。
 *
 * 调用上下文：证据记录和主机测试。
 * 失败行为：参数非法时返回 false。
 */
bool bsp_pwm_dead_time_ns(uint32_t timer_clock_hz, uint32_t encoded, uint32_t *actual_ns)
{
  uint32_t dts_count;

  if ((timer_clock_hz == 0u) || (actual_ns == 0) || (encoded > 0xFFu)) {
    return false;
  }

  if ((encoded & 0x80u) == 0u) {
    dts_count = encoded;
  } else if ((encoded & 0x40u) == 0u) {
    dts_count = 2u * (64u + (encoded & 0x3Fu));
  } else if ((encoded & 0x20u) == 0u) {
    dts_count = 8u * (32u + (encoded & 0x1Fu));
  } else {
    dts_count = 16u * (32u + (encoded & 0x1Fu));
  }

  *actual_ns = bsp_pwm_dts_to_ns(timer_clock_hz, dts_count);
  return true;
}

/**
 * @brief 计算最小脉宽对应的比较值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param prescaler 预分频值。
 * @param min_pulse_ns 最小脉宽约束，单位 ns。
 * @param period_ticks 自动重装值。
 * @param min_pulse_ticks 输出最小脉宽计数。
 * @return true 表示最小脉宽小于一个载波周期。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：最小脉宽不小于自动重装值时返回 false。
 */
bool bsp_pwm_min_pulse_ticks(uint32_t timer_clock_hz,
                             uint32_t prescaler,
                             uint32_t min_pulse_ns,
                             uint32_t period_ticks,
                             uint32_t *min_pulse_ticks)
{
  uint32_t counter_clock;
  uint64_t ticks;

  if ((timer_clock_hz == 0u) || (min_pulse_ticks == 0) || (period_ticks == 0u)) {
    return false;
  }

  counter_clock = timer_clock_hz / (prescaler + 1u);
  if (counter_clock == 0u) {
    return false;
  }

  ticks = bsp_pwm_div_ceil((uint64_t)min_pulse_ns * (uint64_t)counter_clock,
                           BSP_PWM_NANOSECONDS_PER_SECOND);
  if (ticks == 0u) {
    ticks = 1u;
  }
  if (ticks >= (uint64_t)period_ticks) {
    return false;
  }

  *min_pulse_ticks = (uint32_t)ticks;
  return true;
}

/**
 * @brief 判断时基状态是否满足 A4 闸门。
 *
 * @param status 时基状态。
 * @return true 表示中心对齐、载波、死区和最小脉宽全部满足要求。
 *
 * 调用上下文：自检、证据记录和主机测试。
 * 失败行为：状态为空或任一条件不满足时返回 false。
 */
bool bsp_pwm_timing_is_within_limits(const bsp_pwm_status_t *status)
{
  uint32_t difference;

  if (status == 0) {
    return false;
  }
  if (!status->center_aligned) {
    return false;
  }
  if (!status->adc_trigger_armed) {
    return false;
  }
  if ((status->period_ticks == 0u) || (status->period_ticks > BSP_PWM_PERIOD_MAX)) {
    return false;
  }
  if (status->dead_time_ns < BSP_PWM_DEAD_TIME_MIN_NS) {
    return false;
  }
  if ((status->min_pulse_ticks == 0u) || (status->min_pulse_ticks >= status->period_ticks)) {
    return false;
  }

  if (status->carrier_hz > BSP_PWM_CARRIER_HZ) {
    difference = status->carrier_hz - BSP_PWM_CARRIER_HZ;
  } else {
    difference = BSP_PWM_CARRIER_HZ - status->carrier_hz;
  }

  return (difference <= BSP_PWM_CARRIER_TOLERANCE_HZ);
}
