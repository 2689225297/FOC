/**
 * @file bsp_nfault.c
 * @brief DRV8323 nFAULT GPIO 和 EXINT 初始化实现。
 *
 * 主要接口：bsp_nfault_init、bsp_nfault_is_high。
 * 依赖关系：依赖 AT32F403A GPIO/EXINT/MISC 外设库。
 * 关键安全约束：nFAULT 使用最高中断优先级，处理函数不在中断内记日志。
 */

#include "bsp/bsp_nfault.h"

#include "at32f403a_407.h"
#include "bsp/bsp_pin_map.h"

/**
 * @brief 配置 PA15 为带内部上拉的输入。
 *
 * @return 无返回值。
 *
 * 调用上下文：bsp_nfault_init。
 * 失败行为：无失败返回值，后续读取电平用于自检。
 */
static void bsp_nfault_configure_pin(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = BSP_NFAULT_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(BSP_NFAULT_PORT, &gpio_init_struct);
}

/**
 * @brief 初始化 nFAULT 下降沿中断。
 *
 * @return true 表示配置完成。
 *
 * 调用上下文：系统启动、安全管理器初始化之后。
 * 失败行为：无显式厂商错误码；返回 true 后仍需目标板故障注入验证。
 */
bool bsp_nfault_init(void)
{
  exint_init_type exint_init_struct;

  crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
  bsp_nfault_configure_pin();

  gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOA, GPIO_PINS_SOURCE15);
  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
  exint_init_struct.line_select = EXINT_LINE_15;
  exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
  exint_init(&exint_init_struct);

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
  nvic_irq_enable(EXINT15_10_IRQn, 0u, 0u);
  return true;
}

/**
 * @brief 读取 nFAULT 引脚。
 *
 * @return true 表示高电平，false 表示低电平。
 *
 * 调用上下文：自检和诊断。
 * 失败行为：无失败路径。
 */
bool bsp_nfault_is_high(void)
{
  return gpio_input_data_bit_read(BSP_NFAULT_PORT, BSP_NFAULT_PIN) == SET;
}
