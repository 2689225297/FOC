/**
 * @file bsp_pin_map.h
 * @brief AT32F403ACGU7 产品板最终引脚映射。
 *
 * 主要接口：为 BSP 提供唯一引脚常量来源，禁止各模块自行写裸引脚。
 * 依赖关系：依赖厂商 at32f403a_407.h 中的 GPIO 和引脚宏。
 * 关键安全约束：PB12 是 B 路 INLx 总控，任何启动路径都必须先把它配置为低。
 */

#ifndef FOC_BSP_PIN_MAP_H
#define FOC_BSP_PIN_MAP_H

#include "at32f403a_407.h"

/** @brief A 路 U 相高边：PA8，TMR1_CH1。 */
#define BSP_A_U_HIGH_PORT GPIOA
#define BSP_A_U_HIGH_PIN GPIO_PINS_8

/** @brief A 路 V 相高边：PA9，TMR1_CH2。 */
#define BSP_A_V_HIGH_PORT GPIOA
#define BSP_A_V_HIGH_PIN GPIO_PINS_9

/** @brief A 路 W 相高边：PA10，TMR1_CH3。 */
#define BSP_A_W_HIGH_PORT GPIOA
#define BSP_A_W_HIGH_PIN GPIO_PINS_10

/** @brief A 路 U 相低边：PB13，TMR1_CH1C。 */
#define BSP_A_U_LOW_PORT GPIOB
#define BSP_A_U_LOW_PIN GPIO_PINS_13

/** @brief A 路 V 相低边：PB14，TMR1_CH2C。 */
#define BSP_A_V_LOW_PORT GPIOB
#define BSP_A_V_LOW_PIN GPIO_PINS_14

/** @brief A 路 W 相低边：PB15，TMR1_CH3C。 */
#define BSP_A_W_LOW_PORT GPIOB
#define BSP_A_W_LOW_PIN GPIO_PINS_15

/** @brief B 路 U 相：PB0，TMR3_CH3。 */
#define BSP_B_U_PORT GPIOB
#define BSP_B_U_PIN GPIO_PINS_0

/** @brief B 路 V 相：PB1，TMR3_CH4。 */
#define BSP_B_V_PORT GPIOB
#define BSP_B_V_PIN GPIO_PINS_1

/** @brief B 路 W 相：PB7，TMR4_CH2。 */
#define BSP_B_W_PORT GPIOB
#define BSP_B_W_PIN GPIO_PINS_7

/** @brief B 路 DRV8323 INLx 总控：PB12，低电平进入三相 Hi-Z。 */
#define BSP_B_INL_PORT GPIOB
#define BSP_B_INL_PIN GPIO_PINS_12

/** @brief DRV8323 nFAULT：PA15，下降沿外部中断。 */
#define BSP_NFAULT_PORT GPIOA
#define BSP_NFAULT_PIN GPIO_PINS_15

/** @brief 状态灯：PC13。 */
#define BSP_STATUS_LED_PORT GPIOC
#define BSP_STATUS_LED_PIN GPIO_PINS_13

/** @brief A 路 U 相电流：PA0，ADC1 注入通道。 */
#define BSP_A_IU_PORT GPIOA
#define BSP_A_IU_PIN GPIO_PINS_0

/** @brief A 路 V 相电流：PA1，ADC1 注入通道。 */
#define BSP_A_IV_PORT GPIOA
#define BSP_A_IV_PIN GPIO_PINS_1

/** @brief B 路 U 相电流：PA2，ADC2 注入通道。 */
#define BSP_B_IU_PORT GPIOA
#define BSP_B_IU_PIN GPIO_PINS_2

/** @brief B 路 V 相电流：PA3，ADC2 注入通道。 */
#define BSP_B_IV_PORT GPIOA
#define BSP_B_IV_PIN GPIO_PINS_3

/** @brief DRV8323 片选：PA4。 */
#define BSP_DRV_CS_PORT GPIOA
#define BSP_DRV_CS_PIN GPIO_PINS_4

/** @brief DRV8323 SPI 时钟：PA5。 */
#define BSP_DRV_SCLK_PORT GPIOA
#define BSP_DRV_SCLK_PIN GPIO_PINS_5

/** @brief DRV8323 SPI 数据输入：PA6。 */
#define BSP_DRV_SDO_PORT GPIOA
#define BSP_DRV_SDO_PIN GPIO_PINS_6

/** @brief DRV8323 SPI 数据输出：PA7。 */
#define BSP_DRV_SDI_PORT GPIOA
#define BSP_DRV_SDI_PIN GPIO_PINS_7

#endif
