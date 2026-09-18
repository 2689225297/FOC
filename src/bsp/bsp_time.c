/**
 * @file bsp_time.c
 * @brief SysTick 单调毫秒时基实现。
 *
 * 主要接口：bsp_time_init、bsp_time_get_ms、bsp_time_tick_isr。
 * 依赖关系：依赖 CMSIS 内核和 system_core_clock。
 * 关键安全约束：阶段 0 使用 SysTick；接入 FreeRTOS 时必须迁移到 FreeRTOS tick，保持该接口不变。
 */

#include "bsp/bsp_time.h"

#include "at32f403a_407.h"

/** @brief 单调毫秒计数，只在 SysTick 中断写入。 */
static volatile uint32_t g_milliseconds;

/**
 * @brief 初始化 1kHz SysTick。
 *
 * @return true 表示配置成功。
 *
 * 调用上下文：系统时钟初始化后。
 * 失败行为：SysTick_Config 返回非零时返回 false。
 */
bool bsp_time_init(void)
{
  g_milliseconds = 0u;
  return SysTick_Config(system_core_clock / UINT32_C(1000)) == 0u;
}

/**
 * @brief 读取单调毫秒计数。
 *
 * @return 当前毫秒值。
 *
 * 调用上下文：中断和任务。
 * 失败行为：无失败路径。
 */
uint32_t bsp_time_get_ms(void)
{
  return g_milliseconds;
}

/**
 * @brief SysTick 中断递增毫秒计数。
 *
 * @return 无返回值。
 *
 * 调用上下文：仅 SysTick_Handler。
 * 失败行为：无失败路径。
 */
void bsp_time_tick_isr(void)
{
  ++g_milliseconds;
}
