/**
 * @file bsp_log.c
 * @brief UART3 非阻塞调试日志实现。
 *
 * 主要接口：bsp_log_init、bsp_log_write_nonblocking。
 * 依赖关系：依赖 AT32F403A USART/GPIO 外设库。
 * 关键安全约束：任何日志调用都不允许 while 等待发送标志。
 */

#include "bsp/bsp_log.h"

#include "at32f403a_407.h"

/**
 * @brief 初始化 UART3 引脚为复用功能。
 *
 * @return 无返回值。
 *
 * 调用上下文：bsp_log_init。
 * 失败行为：无失败返回值，后续寄存器回读负责诊断。
 */
static void bsp_log_init_pins(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = GPIO_PINS_10 | GPIO_PINS_11;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOB, &gpio_init_struct);
}

/**
 * @brief 初始化 UART3。
 *
 * @return true 表示配置命令完成。
 *
 * 调用上下文：系统启动。
 * 失败行为：厂商库无返回码；目标板需通过回环或示波器验证。
 */
bool bsp_log_init(void)
{
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
  bsp_log_init_pins();

  usart_init(USART3, UINT32_C(921600), USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);
  usart_hardware_flow_control_set(USART3, USART_HARDWARE_FLOW_NONE);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_enable(USART3, TRUE);
  return true;
}

/**
 * @brief 非阻塞发送日志。
 *
 * @param text 字节序列。
 * @param length 字节数。
 * @return true 表示全部发送完成。
 *
 * 调用上下文：低优先级任务。
 * 失败行为：任一字节暂时无法发送时返回 false，不忙等待。
 */
bool bsp_log_write_nonblocking(const char *text, uint32_t length)
{
  uint32_t index;

  if ((text == 0) && (length != 0u)) {
    return false;
  }

  for (index = 0u; index < length; ++index) {
    if (usart_flag_get(USART3, USART_TDBE_FLAG) == RESET) {
      return false;
    }
    usart_data_transmit(USART3, (uint16_t)(uint8_t)text[index]);
  }

  return true;
}
