/**
 * @file bsp_current_sense.h
 * @brief 双轴 A/B 路电流采样链路无功率 BSP 接口。
 *
 * 主要接口：bsp_current_sense_init、bsp_current_sense_read_status、bsp_current_sense_read_raw、
 *          bsp_current_sense_capture_offsets、bsp_current_sense_evaluate_window。
 * 依赖关系：依赖 bsp_pwm.c 配置的 ADC 注入触发、motor_current 的换算与门槛校验、bsp_pin_map.h 的引脚定义。
 * 关键安全约束：零偏采集只在 PWM 输出禁用时进行；量程或窗口校验失败时不得向后传递电流值。
 */

#ifndef FOC_BSP_CURRENT_SENSE_H
#define FOC_BSP_CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 每轴参与换算的采样通道数量。 */
#define BSP_CURRENT_SENSE_CHANNEL_COUNT ((uint32_t)2u)

/** @brief 12 位 ADC 满量程码值。 */
#define BSP_CURRENT_SENSE_ADC_FULL_SCALE ((uint32_t)4095u)

/** @brief 零偏采集样本数量。 */
#define BSP_CURRENT_SENSE_OFFSET_SAMPLES ((uint32_t)64u)

/** @brief 零偏采集过程中允许的最大极差，单位 LSB。 */
#define BSP_CURRENT_SENSE_OFFSET_SPREAD_LIMIT ((uint32_t)8u)

/** @brief 注入采样窗口有效下限，单位 ns。 */
#define BSP_CURRENT_SENSE_MIN_WINDOW_NS ((uint32_t)1000u)

/** @brief ADC 时钟分频系数，与 bsp_pwm.c 的 CRM_ADC_DIV_8 保持一致。 */
#define BSP_CURRENT_SENSE_ADC_DIVIDER ((uint32_t)8u)

/** @brief ADC 采样保持周期数，与 bsp_pwm 的注入序列保持一致。 */
#define BSP_CURRENT_SENSE_SAMPLE_CYCLES ((uint32_t)42u)

/** @brief 12 位转换所需周期数。 */
#define BSP_CURRENT_SENSE_CONVERSION_CYCLES ((uint32_t)13u)

/**
 * @brief 电流采样链路状态快照，用于闸门检查和证据记录。
 */
typedef struct {
  axis_id_t axis;                                                   /**< 轴编号。 */
  uint16_t offset_raw[BSP_CURRENT_SENSE_CHANNEL_COUNT];             /**< 零偏码值。 */
  uint16_t last_raw[BSP_CURRENT_SENSE_CHANNEL_COUNT];               /**< 最近一次注入结果。 */
  uint32_t offset_spread_lsb;                                       /**< 零偏采集极差。 */
  uint32_t adc_clock_hz;                                            /**< ADC 时钟，单位 Hz。 */
  uint32_t conversion_ns;                                           /**< 单次注入转换总时间，单位 ns。 */
  uint32_t sample_window_ns;                                        /**< 有效采样窗口，单位 ns。 */
  bool offsets_valid;                                               /**< true 表示零偏已采集并通过离散度检查。 */
  bool window_valid;                                                /**< true 表示采样窗口满足下限。 */
  bool initialised;                                                 /**< true 表示模块已初始化。 */
} bsp_current_sense_status_t;

/**
 * @brief 初始化电流采样模块并绑定 ADC 注入结果。
 *
 * @return true 表示初始化成功。
 *
 * 调用上下文：app_init，在 bsp_pwm_init 之后。
 * 失败行为：ADC 或 PWM 未就绪时返回 false，不使能任何功率输出。
 */
bool bsp_current_sense_init(void);

/**
 * @brief 读取指定轴的采样链路状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出状态快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：自检、日志和证据记录。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_current_sense_read_status(axis_id_t axis, bsp_current_sense_status_t *status);

/**
 * @brief 读取指定轴某一相电流注入结果。
 *
 * @param axis 轴编号。
 * @param channel_index 通道序号，0 表示第一相，1 表示第二相。
 * @param raw 输出原始码值。
 * @return true 表示读取成功。
 *
 * 调用上下文：控制路径和主机自检。
 * 失败行为：轴号或通道序号非法、模块未初始化时返回 false。
 */
bool bsp_current_sense_read_raw(axis_id_t axis, uint32_t channel_index, uint16_t *raw);

/**
 * @brief 在零电流条件下采集两相零偏码值。
 *
 * @param axis 轴编号。
 * @return true 表示零偏采集通过离散度检查。
 *
 * 调用上下文：A4 无功率验证用例，必须确认功率输出禁用且母线无电流。
 * 失败行为：输出使能、样本离散度过大时返回 false 并清空零偏有效标志。
 */
bool bsp_current_sense_capture_offsets(axis_id_t axis);

/**
 * @brief 查询指定轴零偏是否有效。
 *
 * @param axis 轴编号。
 * @return true 表示零偏已采集且校验通过。
 *
 * 调用上下文：启动检查和故障判定。
 * 失败行为：轴号非法或未初始化时返回 false。
 */
bool bsp_current_sense_offsets_valid(axis_id_t axis);

/**
 * @brief 按当前时基评估采样窗口有效性。
 *
 * @param axis 轴编号。
 * @param carrier_hz 载波频率，单位 Hz。
 * @param dead_time_ns 死区时间，单位 ns。
 * @return true 表示采样窗口不低于下限。
 *
 * 调用上下文：A4 无功率验证用例和自检。
 * 失败行为：窗口不足时返回 false 并清除窗口有效标志。
 */
bool bsp_current_sense_evaluate_window(axis_id_t axis, uint32_t carrier_hz, uint32_t dead_time_ns);

#ifdef __cplusplus
}
#endif

#endif
