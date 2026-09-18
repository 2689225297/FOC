/**
 * @file app_profiling.h
 * @brief 20kHz 控制中断 CPU 占用与周期抖动统计接口。
 *
 * 主要接口：app_profiling_init、app_profiling_tick、app_profiling_get。
 * 依赖关系：仅依赖 C11 固定宽度整数，可在主机测试。
 * 关键安全约束：统计只读不改控制路径；窗口满后滚动覆盖，不产生动态分配。
 */

#ifndef FOC_APP_PROFILING_H
#define FOC_APP_PROFILING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 统计窗口拍数。 */
#define APP_PROFILING_WINDOW_CYCLES ((uint32_t)200u)

/**
 * @brief Profiling 统计快照。
 *
 * 所有周期与忙时计数均以控制时基计数器单位（如 DWT CYCCNT）表示。
 */
typedef struct {
  uint32_t nominal_period_cycles; /**< 额定控制周期，单位 tick。 */
  uint32_t window_count;          /**< 当前窗口已采拍数。 */
  uint32_t busy_cycles;           /**< 当前窗口忙时合计，单位 tick。 */
  uint32_t period_cycles;         /**< 当前窗口周期合计，单位 tick。 */
  uint32_t max_period_cycles;     /**< 当前窗口最大周期，单位 tick。 */
  uint32_t min_period_cycles;     /**< 当前窗口最小周期，单位 tick。 */
  uint32_t overrun_count;         /**< 周期超过额定 110% 的拍数。 */
} app_profiling_report_t;

/**
 * @brief 初始化统计器。
 *
 * @param nominal_period_cycles 额定控制周期，单位 tick，必须大于 0。
 * @return 无返回值。
 *
 * 调用上下文：系统初始化、中断使能前。
 * 失败行为：额定周期为 0 时按 1 处理，保证后续除法安全。
 */
void app_profiling_init(uint32_t nominal_period_cycles);

/**
 * @brief 记录一拍周期与忙时。
 *
 * @param period_cycles 本拍周期，单位 tick。
 * @param busy_cycles 本拍忙时，单位 tick，不得超过 period_cycles。
 * @return 无返回值。
 *
 * 调用上下文：20kHz 控制中断结尾。
 * 失败行为：period_cycles 为 0 时忽略本拍；busy 大于 period 时按 period 截断。
 */
void app_profiling_tick(uint32_t period_cycles, uint32_t busy_cycles);

/**
 * @brief 清空统计窗口。
 *
 * @return 无返回值。
 *
 * 调用上下文：闸门测量开始前。
 * 失败行为：无失败路径。
 */
void app_profiling_reset(void);

/**
 * @brief 读取统计快照。
 *
 * @param report 输出快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：1kHz 诊断任务和闸门证据。
 * 失败行为：指针为空返回 false。
 */
bool app_profiling_get(app_profiling_report_t *report);

/**
 * @brief 计算当前窗口 CPU 占用百分比。
 *
 * @return 0 到 100 的整数百分比；窗口为空返回 0。
 *
 * 调用上下文：诊断和闸门证据。
 * 失败行为：无失败路径。
 */
uint32_t app_profiling_cpu_percent(void);

/**
 * @brief 计算当前窗口周期抖动百分比。
 *
 * @return 抖动百分比 = (max-min)/nominal*100；窗口不足两拍返回 0。
 *
 * 调用上下文：诊断和闸门证据。
 * 失败行为：无失败路径。
 */
uint32_t app_profiling_jitter_percent(void);

#ifdef __cplusplus
}
#endif

#endif
