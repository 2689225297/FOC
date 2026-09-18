/**
 * @file bsp_nfault.h
 * @brief DRV8323 nFAULT 输入和外部中断接口。
 *
 * 主要接口：初始化 PA15 下降沿中断和读取引脚电平。
 * 依赖关系：依赖 AT32F403A EXINT/GPIO 外设库。
 * 关键安全约束：中断处理只允许执行安全输出、故障锁存和状态机事件入队。
 */

#ifndef FOC_BSP_NFAULT_H
#define FOC_BSP_NFAULT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 PA15 上拉输入和下降沿外部中断。
 *
 * @return true 表示初始化完成。
 *
 * 调用上下文：安全故障管理器和 GPIO 时钟初始化之后。
 * 失败行为：返回 false 时禁止使能带功率运行。
 */
bool bsp_nfault_init(void);

/**
 * @brief 读取 nFAULT 去抖前的引脚电平。
 *
 * @return true 表示引脚为高，false 表示引脚为低。
 *
 * 调用上下文：自检和慢速诊断。
 * 失败行为：无失败路径。
 */
bool bsp_nfault_is_high(void);

#ifdef __cplusplus
}
#endif

#endif
