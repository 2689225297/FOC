/**
 * @file bsp_watchdog.h
 * @brief 独立看门狗接口。
 *
 * 主要接口：初始化和喂狗。
 * 依赖关系：依赖 AT32F403A WDT 外设库。
 * 关键安全约束：看门狗超时复位后必须保持 A/B 路输出关闭并进入可诊断安全状态。
 */

#ifndef FOC_BSP_WATCHDOG_H
#define FOC_BSP_WATCHDOG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化独立看门狗。
 *
 * @return true 表示配置成功。
 *
 * 调用上下文：系统时钟和安全 GPIO 完成后、主循环前。
 * 失败行为：无显式错误码时仍由状态自检确认寄存器；返回 false 时不得运行电机。
 */
bool bsp_watchdog_init(void);

/**
 * @brief 重新装载看门狗计数器。
 *
 * @return 无返回值。
 *
 * 调用上下文：仅在确认所有关键 1kHz 工作完成后调用。
 * 失败行为：无失败路径。
 */
void bsp_watchdog_reload(void);

#ifdef __cplusplus
}
#endif

#endif
