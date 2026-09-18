/**
 * @file bsp_safe_outputs.c
 * @brief GPIO 上电安全态和功率输出快速关闭实现。
 *
 * 主要接口：bsp_safe_outputs_early、bsp_safe_outputs、bsp_gpio_init。
 * 依赖关系：依赖 AT32F403A 标准外设库和 bsp_pin_map.h。
 * 关键安全约束：PB12 必须在任何 SPI、PWM 或 DRV8323 操作之前配置为低。
 */

#include "bsp/bsp_safe_outputs.h"

#include "at32f403a_407.h"
#include "bsp/bsp_pin_map.h"

/**
 * @brief 使能安全初始化所需的 GPIO 和定时器时钟。
 *
 * @return 无返回值。
 *
 * 调用上下文：上电初始化和快速安全路径。
 * 失败行为：无失败返回值；外设时钟若不可用由后续自检标记。
 */
static void bsp_safe_outputs_enable_clocks(void)
{
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
}

/**
 * @brief 把指定引脚配置为输入高阻。
 *
 * @param port GPIO 端口。
 * @param pins 引脚掩码。
 * @return 无返回值。
 *
 * 调用上下文：安全初始化和外部 PWM 模式。
 * 失败行为：厂商库配置失败时由上电自检的回读检查发现。
 */
static void bsp_configure_input(gpio_type *port, uint16_t pins)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = pins;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(port, &gpio_init_struct);
}

/**
 * @brief 把指定引脚配置为推挽输出。
 *
 * @param port GPIO 端口。
 * @param pins 引脚掩码。
 * @param initial_level true 表示初值为高，false 表示初值为低。
 * @return 无返回值。
 *
 * 调用上下文：PB12 和状态灯初始化。
 * 失败行为：电平写入失败由启动回读或示波器验证发现。
 */
static void bsp_configure_output(gpio_type *port, uint16_t pins, bool initial_level)
{
  gpio_init_type gpio_init_struct;

  if (initial_level) {
    gpio_bits_set(port, pins);
  } else {
    gpio_bits_reset(port, pins);
  }

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_pins = pins;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(port, &gpio_init_struct);

  if (initial_level) {
    gpio_bits_set(port, pins);
  } else {
    gpio_bits_reset(port, pins);
  }
}

/**
 * @brief 在时钟重配置前把 PB12 和两路 PWM 输入置为安全态。
 *
 * @return 无返回值。
 *
 * 调用上下文：main 第一天调用，必须在 system_clock_config 之前。
 * 失败行为：无失败返回；目标板必须用示波器验证复位窗口没有有效 PWM。
 */
void bsp_safe_outputs_early(void)
{
  bsp_safe_outputs_enable_clocks();

  /* PB12 是第一优先动作：低电平使 DRV8323 三相桥臂进入 Hi-Z。 */
  bsp_configure_output(BSP_B_INL_PORT, BSP_B_INL_PIN, false);

  bsp_configure_input(BSP_A_U_HIGH_PORT, BSP_A_U_HIGH_PIN);
  bsp_configure_input(BSP_A_V_HIGH_PORT, BSP_A_V_HIGH_PIN);
  bsp_configure_input(BSP_A_W_HIGH_PORT, BSP_A_W_HIGH_PIN);
  bsp_configure_input(BSP_A_U_LOW_PORT, BSP_A_U_LOW_PIN);
  bsp_configure_input(BSP_A_V_LOW_PORT, BSP_A_V_LOW_PIN);
  bsp_configure_input(BSP_A_W_LOW_PORT, BSP_A_W_LOW_PIN);

  bsp_configure_input(BSP_B_U_PORT, BSP_B_U_PIN);
  bsp_configure_input(BSP_B_V_PORT, BSP_B_V_PIN);
  bsp_configure_input(BSP_B_W_PORT, BSP_B_W_PIN);
}

/**
 * @brief 立即关闭两路功率输出。
 *
 * @return 无返回值。
 *
 * 调用上下文：状态进入安全状态、控制快速保护或 nFAULT 中断。
 * 失败行为：仅执行直接寄存器动作，不返回错误，不等待任何任务。
 */
void bsp_safe_outputs(void)
{
  bsp_safe_outputs_enable_clocks();

  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
  tmr_output_enable(TMR3, FALSE);
  tmr_counter_enable(TMR3, FALSE);
  tmr_output_enable(TMR4, FALSE);
  tmr_counter_enable(TMR4, FALSE);

  gpio_bits_reset(BSP_A_U_HIGH_PORT, BSP_A_U_HIGH_PIN);
  gpio_bits_reset(BSP_A_V_HIGH_PORT, BSP_A_V_HIGH_PIN);
  gpio_bits_reset(BSP_A_W_HIGH_PORT, BSP_A_W_HIGH_PIN);
  gpio_bits_reset(BSP_A_U_LOW_PORT, BSP_A_U_LOW_PIN);
  gpio_bits_reset(BSP_A_V_LOW_PORT, BSP_A_V_LOW_PIN);
  gpio_bits_reset(BSP_A_W_LOW_PORT, BSP_A_W_LOW_PIN);

  gpio_bits_reset(BSP_B_U_PORT, BSP_B_U_PIN);
  gpio_bits_reset(BSP_B_V_PORT, BSP_B_V_PIN);
  gpio_bits_reset(BSP_B_W_PORT, BSP_B_W_PIN);
  gpio_bits_reset(BSP_B_INL_PORT, BSP_B_INL_PIN);
}

/**
 * @brief 初始化状态灯并再次确认安全输出。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统时钟稳定后。
 * 失败行为：初始化后保持输出关闭。
 */
void bsp_gpio_init(void)
{
  bsp_safe_outputs();
  bsp_configure_output(BSP_STATUS_LED_PORT, BSP_STATUS_LED_PIN, false);
}

/**
 * @brief 设置状态灯。
 *
 * @param enabled true 点亮，false 熄灭。
 * @return 无返回值。
 *
 * 调用上下文：慢速任务。
 * 失败行为：无失败路径。
 */
void bsp_status_led_set(bool enabled)
{
  if (enabled) {
    gpio_bits_set(BSP_STATUS_LED_PORT, BSP_STATUS_LED_PIN);
  } else {
    gpio_bits_reset(BSP_STATUS_LED_PORT, BSP_STATUS_LED_PIN);
  }
}
