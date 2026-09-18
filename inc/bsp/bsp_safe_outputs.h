/**
 * @file bsp_safe_outputs.h
 * @brief 功率输出安全态接口。
 *
 * 主要接口：最早安全初始化、运行时安全输出、普通 GPIO 初始化和状态灯控制。
 * 依赖关系：依赖 at32f403a_407.h 和 bsp_pin_map.h。
 * 关键安全约束：bsp_safe_outputs_early 必须在使用任何驱动或 PWM 初始化之前调用。
 */

#ifndef FOC_BSP_SAFE_OUTPUTS_H
#define FOC_BSP_SAFE_OUTPUTS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在系统时钟重配置前建立最低限度安全态。
 *
 * @return 无返回值。
 *
 * 调用上下文：main 的第一条产品初始化语句。
 * 失败行为：无失败返回值；任何外设配置失败最终由后续自检发现。
 */
void bsp_safe_outputs_early(void);

/**
 * @brief 立即关闭两路功率输出。
 *
 * @return 无返回值。
 *
 * 调用上下文：中断和任务均可调用。
 * 失败行为：本函数只访问已经使能时钟的寄存器，不等待、不分配内存。
 */
void bsp_safe_outputs(void);

/**
 * @brief 初始化状态灯和通用安全 GPIO。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统时钟稳定后、中断使能前。
 * 失败行为：初始化后仍保持 A/B 路功率输出无效。
 */
void bsp_gpio_init(void);

/**
 * @brief 设置状态灯。
 *
 * @param enabled true 点亮，false 熄灭。
 * @return 无返回值。
 *
 * 调用上下文：慢速任务和诊断。
 * 失败行为：无失败路径。
 */
void bsp_status_led_set(bool enabled);

#ifdef __cplusplus
}
#endif

#endif
