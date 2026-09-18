/**
 * @file app_profiling.c
 * @brief 20kHz 控制中断 CPU 占用与周期抖动统计实现。
 *
 * 使用环形窗口保存周期与忙时，采用 64 位累加防止长时间运行溢出。
 */

#include "app/app_profiling.h"

/** @brief 窗口周期数组。 */
static uint32_t g_period_window[APP_PROFILING_WINDOW_CYCLES];

/** @brief 窗口忙时数组。 */
static uint32_t g_busy_window[APP_PROFILING_WINDOW_CYCLES];

/** @brief 环形写索引。 */
static uint32_t g_window_index;

/** @brief 当前已采拍数。 */
static uint32_t g_window_count;

/** @brief 额定周期，单位 tick。 */
static uint32_t g_nominal_period_cycles;

/** @brief 窗口周期累加，单位 tick。 */
static uint64_t g_period_sum;

/** @brief 窗口忙时累加，单位 tick。 */
static uint64_t g_busy_sum;

/** @brief 窗口最大周期。 */
static uint32_t g_max_period_cycles;

/** @brief 窗口最小周期。 */
static uint32_t g_min_period_cycles;

/** @brief 周期超过额定 110% 的拍数。 */
static uint32_t g_overrun_count;

/**
 * @brief 初始化统计器。
 *
 * @param nominal_period_cycles 额定控制周期，单位 tick。
 * @return 无返回值。
 */
void app_profiling_init(uint32_t nominal_period_cycles)
{
  uint32_t index;

  g_nominal_period_cycles = (nominal_period_cycles == 0u) ? 1u : nominal_period_cycles;

  for (index = 0u; index < APP_PROFILING_WINDOW_CYCLES; ++index) {
    g_period_window[index] = 0u;
    g_busy_window[index] = 0u;
  }

  g_window_index = 0u;
  g_window_count = 0u;
  g_period_sum = 0u;
  g_busy_sum = 0u;
  g_max_period_cycles = 0u;
  g_min_period_cycles = 0u;
  g_overrun_count = 0u;
}

/**
 * @brief 记录一拍周期与忙时。
 *
 * @param period_cycles 本拍周期，单位 tick。
 * @param busy_cycles 本拍忙时，单位 tick。
 * @return 无返回值。
 */
void app_profiling_tick(uint32_t period_cycles, uint32_t busy_cycles)
{
  uint32_t busy_clamped;
  uint32_t replaced_period;
  uint32_t replaced_busy;

  if (period_cycles == 0u) {
    return;
  }

  busy_clamped = (busy_cycles > period_cycles) ? period_cycles : busy_cycles;

  replaced_period = g_period_window[g_window_index];
  replaced_busy = g_busy_window[g_window_index];

  g_period_window[g_window_index] = period_cycles;
  g_busy_window[g_window_index] = busy_clamped;

  g_period_sum = g_period_sum - replaced_period + period_cycles;
  g_busy_sum = g_busy_sum - replaced_busy + busy_clamped;

  if (g_window_count < APP_PROFILING_WINDOW_CYCLES) {
    ++g_window_count;
  }

  if ((g_window_count == 1u) || (period_cycles > g_max_period_cycles)) {
    g_max_period_cycles = period_cycles;
  }
  if ((g_window_count == 1u) || (period_cycles < g_min_period_cycles)) {
    g_min_period_cycles = period_cycles;
  }

  if (period_cycles > (g_nominal_period_cycles + (g_nominal_period_cycles / 10u))) {
    ++g_overrun_count;
  }

  g_window_index = (g_window_index + 1u) % APP_PROFILING_WINDOW_CYCLES;
}

/**
 * @brief 清空统计窗口。
 *
 * @return 无返回值。
 */
void app_profiling_reset(void)
{
  app_profiling_init(g_nominal_period_cycles);
}

/**
 * @brief 读取统计快照。
 *
 * @param report 输出快照。
 * @return true 表示读取成功。
 */
bool app_profiling_get(app_profiling_report_t *report)
{
  if (report == 0) {
    return false;
  }

  report->nominal_period_cycles = g_nominal_period_cycles;
  report->window_count = g_window_count;
  report->busy_cycles = (uint32_t)g_busy_sum;
  report->period_cycles = (uint32_t)g_period_sum;
  report->max_period_cycles = g_max_period_cycles;
  report->min_period_cycles = g_min_period_cycles;
  report->overrun_count = g_overrun_count;
  return true;
}

/**
 * @brief 计算当前窗口 CPU 占用百分比。
 *
 * @return 0 到 100 的整数百分比。
 */
uint32_t app_profiling_cpu_percent(void)
{
  uint32_t busy = (uint32_t)g_busy_sum;
  uint32_t period = (uint32_t)g_period_sum;

  if (period == 0u) {
    return 0u;
  }

  return (uint32_t)(((uint64_t)busy * 100u) / (uint64_t)period);
}

/**
 * @brief 计算当前窗口周期抖动百分比。
 *
 * @return 抖动百分比。
 */
uint32_t app_profiling_jitter_percent(void)
{
  uint32_t jitter;
  uint32_t percent;

  if (g_window_count < 2u) {
    return 0u;
  }

  jitter = g_max_period_cycles - g_min_period_cycles;
  percent = (uint32_t)(((uint64_t)jitter * 100u) / (uint64_t)g_nominal_period_cycles);
  return percent;
}
