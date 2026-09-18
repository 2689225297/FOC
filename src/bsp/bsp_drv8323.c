/**
 * @file bsp_drv8323.c
 * @brief DRV8323 阶段 0 安全状态实现。
 *
 * 主要接口：bsp_drv8323_init_safe、bsp_drv8323_is_calibrated、bsp_drv8323_nfault_is_clear。
 * 依赖关系：依赖 bsp_nfault 和 bsp_safe_outputs。
 * 关键安全约束：完整 SPI 校准尚未接入前，校准状态固定为 false，B 路不得使能。
 */

#include "bsp/bsp_drv8323.h"

#include "bsp/bsp_nfault.h"
#include "bsp/bsp_safe_outputs.h"

/** @brief DRV8323 CSA 校准完成标志，阶段 0 保持无效。 */
static volatile bool g_drv8323_calibrated;

/**
 * @brief 初始化 DRV8323 安全状态。
 *
 * @return true 表示安全态初始化完成。
 *
 * 调用上下文：系统启动。
 * 失败行为：任何路径都不开启 PWM；校准标志保持 false。
 */
bool bsp_drv8323_init_safe(void)
{
  bsp_safe_outputs();
  g_drv8323_calibrated = false;
  return bsp_nfault_init();
}

/**
 * @brief 查询 CSA 校准状态。
 *
 * @return 当前校准完成状态；阶段 0 固定为 false。
 *
 * 调用上下文：状态机和自检。
 * 失败行为：无失败路径。
 */
bool bsp_drv8323_is_calibrated(void)
{
  return g_drv8323_calibrated;
}

/**
 * @brief 查询 nFAULT 是否清除。
 *
 * @return true 表示 PA15 为高。
 *
 * 调用上下文：自检和恢复。
 * 失败行为：无失败路径。
 */
bool bsp_drv8323_nfault_is_clear(void)
{
  return bsp_nfault_is_high();
}
