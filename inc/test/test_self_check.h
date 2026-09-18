/**
 * @file test_self_check.h
 * @brief 阶段 0 上电自检接口。
 *
 * 主要接口：检查编译模型、参数安全状态、nFAULT 和 DRV8323 校准状态。
 * 依赖关系：依赖电机类型、BSP 和参数存储模块。
 * 关键安全约束：任何不确定结果必须返回 false，禁止放行带功率运行。
 */

#ifndef FOC_TEST_SELF_CHECK_H
#define FOC_TEST_SELF_CHECK_H

#include <stdbool.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行指定轴的阶段 0 自检。
 *
 * @param axis 轴编号。
 * @return true 表示静态模型安全且该轴允许进入下一安全校准阶段。
 *
 * 调用上下文：1kHz 慢速任务，仅在 SELF_TEST 状态调用。
 * 失败行为：非法轴、nFAULT 低或编译模型不满足要求时返回 false。
 */
bool test_self_check_run(axis_id_t axis);

#ifdef __cplusplus
}
#endif

#endif
