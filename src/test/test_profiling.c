/**
 * @file test_profiling.c
 * @brief Cortex-M4 DWT Profiling 实现。
 *
 * 主要接口：test_profiling_init、test_profiling_read_cycles、
 * test_profiling_elapsed_cycles。
 * 依赖关系：依赖 AT32F403A CMSIS 内核头文件。
 * 关键安全约束：周期计数只用于测量，不参与故障判断和安全动作。
 */

#include "test/test_profiling.h"

#include "at32f403a_407.h"

/**
 * @brief 初始化 DWT 周期计数器。
 *
 * @return true 表示计数器已使能。
 *
 * 调用上下文：调试和性能测试。
 * 失败行为：无法访问 DWT 时返回 false。
 */
bool test_profiling_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u;
}

/**
 * @brief 读取 DWT 周期计数。
 *
 * @return 当前周期计数。
 *
 * 调用上下文：性能测量。
 * 失败行为：无失败路径。
 */
uint32_t test_profiling_read_cycles(void)
{
  return DWT->CYCCNT;
}

/**
 * @brief 计算周期差。
 *
 * @param start_cycles 起始周期。
 * @param end_cycles 结束周期。
 * @return 允许回绕的周期差。
 *
 * 调用上下文：性能统计。
 * 失败行为：无失败路径。
 */
uint32_t test_profiling_elapsed_cycles(uint32_t start_cycles, uint32_t end_cycles)
{
  return end_cycles - start_cycles;
}
