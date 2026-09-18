/**
 * @file bsp_time.h
 * @brief 单调毫秒时基接口。
 *
 * 主要接口：初始化 SysTick、读取毫秒时间和由中断递增时基。
 * 依赖关系：依赖 CMSIS 内核接口。
 * 关键安全约束：故障时间戳必须来自单调时基，禁止在中断中读取阻塞时钟。
 */

#ifndef FOC_BSP_TIME_H
#define FOC_BSP_TIME_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 1kHz SysTick 时基。
 *
 * @return true 表示 SysTick 配置成功。
 *
 * 调用上下文：系统时钟完成配置后。
 * 失败行为：失败时返回 false，调用方必须进入安全停机。
 */
bool bsp_time_init(void);

/**
 * @brief 读取单调毫秒时间。
 *
 * @return 从复位或计数器回绕后的毫秒值，单位 ms。
 *
 * 调用上下文：中断和任务。
 * 失败行为：无失败路径；32 位值约 49.7 天回绕。
 */
uint32_t bsp_time_get_ms(void);

/**
 * @brief 由 SysTick 中断递增毫秒计数。
 *
 * @return 无返回值。
 *
 * 调用上下文：仅 SysTick_Handler。
 * 失败行为：无失败路径。
 */
void bsp_time_tick_isr(void);

#ifdef __cplusplus
}
#endif

#endif
