/**
 * @file bsp_clock.h
 * @brief 系统时钟配置接口。
 *
 * 主要接口：把 AT32F403ACGU7 配置为 192MHz 并返回状态。
 * 依赖关系：依赖 AT32F403A 标准外设库。
 * 关键安全约束：时钟初始化失败时不得启动电机或使能功率输出。
 */

#ifndef FOC_BSP_CLOCK_H
#define FOC_BSP_CLOCK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配置 8MHz HEXT、PLL 和 192MHz 系统时钟。
 *
 * @return true 表示时钟切换和全局时钟变量更新完成。
 *
 * 调用上下文：安全 GPIO 早期初始化之后、其他外设初始化之前。
 * 失败行为：HEXT/PLL 超时返回 false，调用方必须进入安全停机。
 */
bool bsp_clock_init(void);

/**
 * @brief 读取并清除看门狗复位标志。
 *
 * @return true 表示本次复位由独立看门狗触发。
 *
 * 调用上下文：系统启动早期，必须在其他模块清除 CRM 标志前调用。
 * 失败行为：无失败路径；读取后清除该一次性标志。
 */
bool bsp_clock_take_watchdog_reset_flag(void);

#ifdef __cplusplus
}
#endif

#endif
