/**
 * @file bsp_drv8323.h
 * @brief DRV8323 安全状态和校准状态接口。
 *
 * 主要接口：初始化为安全未校准状态、查询 nFAULT 和校准状态。
 * 依赖关系：依赖 bsp_nfault.h；完整 SPI 配置在后续阶段接入。
 * 关键安全约束：未完成 SPI 回读和 CSA_CAL_x 校准时，校准状态必须保持无效。
 */

#ifndef FOC_BSP_DRV8323_H
#define FOC_BSP_DRV8323_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 DRV8323 软件状态并保持 PB12 低。
 *
 * @return true 表示已经进入安全的未校准状态。
 *
 * 调用上下文：GPIO 安全初始化之后、任何功率使能之前。
 * 失败行为：始终不使能 PWM；返回 false 表示后续 BSP 初始化失败。
 */
bool bsp_drv8323_init_safe(void);

/**
 * @brief 查询 DRV8323 CSA 校准状态。
 *
 * @return true 表示 SPI 配置、CSA_CAL_x 和寄存器回读均已通过。
 *
 * 调用上下文：状态机进入 IDLE 前。
 * 失败行为：未完成完整校准流程时返回 false。
 */
bool bsp_drv8323_is_calibrated(void);

/**
 * @brief 查询 nFAULT 引脚是否已恢复为高。
 *
 * @return true 表示当前无低电平故障。
 *
 * 调用上下文：自检和恢复流程。
 * 失败行为：无失败路径。
 */
bool bsp_drv8323_nfault_is_clear(void);

#ifdef __cplusplus
}
#endif

#endif
