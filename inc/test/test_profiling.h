/**
 * @file test_profiling.h
 * @brief DWT 周期计数 Profiling 钩子。
 *
 * 主要接口：初始化、开始计数、停止并读取周期差。
 * 依赖关系：依赖 Cortex-M4 DWT 和 CoreDebug。
 * 关键安全约束：Profiling 不得改变控制逻辑；Release 中仅保留显式调用点。
 */

#ifndef FOC_TEST_PROFILING_H
#define FOC_TEST_PROFILING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 DWT 周期计数器。
 *
 * @return true 表示周期计数已使能。
 *
 * 调用上下文：调试和性能测试初始化。
 * 失败行为：返回 false 时不使用周期计数结果。
 */
bool test_profiling_init(void);

/**
 * @brief 读取当前 DWT 周期计数。
 *
 * @return 32 位周期计数；未初始化时行为由硬件决定。
 *
 * 调用上下文：测量开始和结束。
 * 失败行为：无失败路径。
 */
uint32_t test_profiling_read_cycles(void);

/**
 * @brief 计算两个周期计数的差。
 *
 * @param start_cycles 起始周期。
 * @param end_cycles 结束周期。
 * @return 无符号差值，允许 32 位计数器回绕。
 *
 * 调用上下文：性能统计。
 * 失败行为：无失败路径。
 */
uint32_t test_profiling_elapsed_cycles(uint32_t start_cycles, uint32_t end_cycles);

#ifdef __cplusplus
}
#endif

#endif
