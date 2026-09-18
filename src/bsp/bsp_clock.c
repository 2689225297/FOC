/**
 * @file bsp_clock.c
 * @brief 192MHz 系统时钟实现。
 *
 * 主要接口：bsp_clock_init。
 * 依赖关系：依赖 AT32F403A CRM 外设库。
 * 关键安全约束：任何时钟切换超时都必须返回失败，禁止继续初始化功率外设。
 */

#include "bsp/bsp_clock.h"

#include "at32f403a_407.h"

/** @brief HEXT 稳定等待循环次数。 */
#define BSP_HEXT_STABLE_DELAY UINT32_C(0x2000)

/** @brief PLL 分频切换等待循环次数。 */
#define BSP_PLL_STABLE_DELAY UINT32_C(0x2000)

/** @brief HEXT 和 PLL 等待上限，单位循环次数。 */
#define BSP_CLOCK_WAIT_LIMIT UINT32_C(1000000)

/**
 * @brief 执行短硬件稳定等待。
 *
 * @param delay 循环次数。
 * @return 无返回值。
 *
 * 调用上下文：系统时钟初始化。
 * 失败行为：无失败路径。
 */
static void bsp_clock_short_delay(uint32_t delay)
{
  volatile uint32_t index;

  for (index = 0u; index < delay; ++index) {
    /* 空循环用于满足时钟源和分频切换的稳定等待要求。 */
  }
}

/**
 * @brief 配置 192MHz 系统时钟。
 *
 * @return true 表示成功；false 表示 HEXT 或 PLL 稳定等待超时。
 *
 * 调用上下文：安全 GPIO 早期初始化之后。
 * 失败行为：失败时保留安全 GPIO，不继续初始化电机外设。
 */
bool bsp_clock_init(void)
{
  uint32_t wait_count;

  crm_reset();
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);
  bsp_clock_short_delay(BSP_HEXT_STABLE_DELAY);

  wait_count = 0u;
  while (crm_hext_stable_wait() == ERROR) {
    if (++wait_count >= BSP_CLOCK_WAIT_LIMIT) {
      return false;
    }
  }

  crm_pll_config(CRM_PLL_SOURCE_HEXT_DIV, CRM_PLL_MULT_48, CRM_PLL_OUTPUT_RANGE_GT72MHZ);
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  wait_count = 0u;
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
    if (++wait_count >= BSP_CLOCK_WAIT_LIMIT) {
      return false;
    }
  }

  crm_apb2_div_set(CRM_APB2_DIV_2);
  crm_apb1_div_set(CRM_APB1_DIV_2);
  crm_ahb_div_set(CRM_AHB_DIV_8);
  crm_sysclk_switch(CRM_SCLK_PLL);

  wait_count = 0u;
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
    if (++wait_count >= BSP_CLOCK_WAIT_LIMIT) {
      return false;
    }
  }

  bsp_clock_short_delay(BSP_PLL_STABLE_DELAY);
  crm_ahb_div_set(CRM_AHB_DIV_2);
  bsp_clock_short_delay(BSP_PLL_STABLE_DELAY);
  crm_ahb_div_set(CRM_AHB_DIV_1);
  system_core_clock_update();

  return true;
}

/**
 * @brief 读取并清除独立看门狗复位标志。
 *
 * @return true 表示检测到看门狗复位。
 *
 * 调用上下文：系统启动早期。
 * 失败行为：无失败路径。
 */
bool bsp_clock_take_watchdog_reset_flag(void)
{
  bool watchdog_reset = crm_flag_get(CRM_WDT_RESET_FLAG) == SET;

  if (watchdog_reset) {
    crm_flag_clear(CRM_WDT_RESET_FLAG);
  }

  return watchdog_reset;
}
