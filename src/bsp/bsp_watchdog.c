/**
 * @file bsp_watchdog.c
 * @brief 独立看门狗实现。
 *
 * 主要接口：bsp_watchdog_init、bsp_watchdog_reload。
 * 依赖关系：依赖 AT32F403A WDT 外设库。
 * 关键安全约束：阶段 0 的喂狗只允许发生在完整 1kHz 服务之后，禁止在故障循环中无条件喂狗。
 */

#include "bsp/bsp_watchdog.h"

#include "at32f403a_407.h"

/**
 * @brief 初始化约 500ms 超时的独立看门狗。
 *
 * @return true 表示初始化命令完成。
 *
 * 调用上下文：系统启动。
 * 失败行为：厂商库接口无返回码；上电自检需增加寄存器回读项。
 */
bool bsp_watchdog_init(void)
{
  /* HICK 约 40kHz 经 64 分频为约 625Hz，312 个计数约等于 500ms。 */
  wdt_register_write_enable(TRUE);
  wdt_divider_set(WDT_CLK_DIV_64);
  wdt_reload_value_set(UINT16_C(312));
  wdt_enable();
  return true;
}

/**
 * @brief 重新装载看门狗计数器。
 *
 * @return 无返回值。
 *
 * 调用上下文：完整 1kHz 服务周期末尾。
 * 失败行为：无失败路径。
 */
void bsp_watchdog_reload(void)
{
  wdt_counter_reload();
}
