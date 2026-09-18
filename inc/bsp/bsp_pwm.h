/**
 * @file bsp_pwm.h
 * @brief 双轴 20kHz 中心对齐 PWM 时基、死区与 ADC 注入触发接口。
 *
 * 主要接口：bsp_pwm_init、bsp_pwm_read_status、bsp_pwm_read_registers、
 *          bsp_pwm_outputs_enabled、bsp_pwm_test_enable_outputs。
 * 依赖关系：依赖 motor_types.h 的 axis_id_t；纯计算部分不依赖厂商头文件。
 * 关键安全约束：初始化只配置时基与触发，结束时保持两路输出禁用；
 *              任何输出使能动作都由 bsp_safe_outputs 门控，并且必须在无功率条件下进行。
 */

#ifndef FOC_BSP_PWM_H
#define FOC_BSP_PWM_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 载波频率目标值，单位 Hz。 */
#define BSP_PWM_CARRIER_HZ ((uint32_t)20000u)

/** @brief 载波频率允许偏差，单位 Hz，对应 1%。 */
#define BSP_PWM_CARRIER_TOLERANCE_HZ ((uint32_t)200u)

/** @brief 死区时间下限，单位 ns。 */
#define BSP_PWM_DEAD_TIME_MIN_NS ((uint32_t)500u)

/** @brief 中心对齐时基下的最小有效脉宽，单位 ns。 */
#define BSP_PWM_MIN_PULSE_NS ((uint32_t)1000u)

/** @brief 时基预分频搜索上限。 */
#define BSP_PWM_PRESCALER_MAX ((uint32_t)65535u)

/** @brief 计数器自动重装上限。 */
#define BSP_PWM_PERIOD_MAX ((uint32_t)65535u)

/**
 * @brief PWM 时基状态快照，用于闸门检查和证据记录。
 */
typedef struct {
  axis_id_t axis;             /**< 轴编号。 */
  uint32_t timer_clock_hz;    /**< 定时器计数时钟，单位 Hz。 */
  uint32_t carrier_hz;        /**< 实际载波频率，单位 Hz。 */
  uint32_t period_ticks;      /**< 自动重装值。 */
  uint32_t prescaler;         /**< 预分频值。 */
  uint32_t dead_time_encoded; /**< 死区发生器编码值。 */
  uint32_t dead_time_ns;      /**< 死区时间换算值，单位 ns。 */
  uint32_t min_pulse_ticks;   /**< 最小脉宽计数，单位 tick。 */
  bool center_aligned;        /**< true 表示中心对齐模式。 */
  bool counter_running;       /**< true 表示计数器正在运行。 */
  bool outputs_enabled;       /**< true 表示功率输出通道已使能。 */
  bool adc_trigger_armed;     /**< true 表示 ADC 注入触发已配置。 */
  bool subordinate_synced;    /**< true 表示从定时器已配置同步输入。 */
} bsp_pwm_status_t;

/**
 * @brief 按 APB 与 AHB 频率计算定时器计数时钟。
 *
 * @param ahb_freq_hz AHB 频率，单位 Hz。
 * @param apb_freq_hz APB 频率，单位 Hz。
 * @param timer_clock_hz 输出定时器计数时钟，单位 Hz。
 * @return true 表示计算成功。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：参数非法或频率为零时返回 false，输出保持不变。
 */
bool bsp_pwm_timer_clock_hz(uint32_t ahb_freq_hz,
                            uint32_t apb_freq_hz,
                            uint32_t *timer_clock_hz);

/**
 * @brief 计算中心对齐时基的预分频和自动重装值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param carrier_hz 目标载波频率，单位 Hz。
 * @param prescaler 输出预分频值。
 * @param period_ticks 输出自动重装值。
 * @param actual_carrier_hz 输出实际载波频率，单位 Hz。
 * @return true 表示找到满足分辨率要求的时基组合。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：无法在计数器范围内达到目标频率时返回 false。
 */
bool bsp_pwm_compute_timing(uint32_t timer_clock_hz,
                            uint32_t carrier_hz,
                            uint32_t *prescaler,
                            uint32_t *period_ticks,
                            uint32_t *actual_carrier_hz);

/**
 * @brief 把死区时间编码为定时器死区发生器取值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param dead_time_ns 目标死区时间，单位 ns。
 * @param encoded 输出死区编码值。
 * @param actual_ns 输出实际死区时间，单位 ns。
 * @return true 表示编码落在定时器支持范围内。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：请求值超过最大可编码死区时返回 false。
 */
bool bsp_pwm_encode_dead_time(uint32_t timer_clock_hz,
                              uint32_t dead_time_ns,
                              uint32_t *encoded,
                              uint32_t *actual_ns);

/**
 * @brief 把死区编码值反算为时间。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param encoded 死区编码值。
 * @param actual_ns 输出实际死区时间，单位 ns。
 * @return true 表示反算成功。
 *
 * 调用上下文：证据记录和主机测试。
 * 失败行为：参数非法时返回 false。
 */
bool bsp_pwm_dead_time_ns(uint32_t timer_clock_hz, uint32_t encoded, uint32_t *actual_ns);

/**
 * @brief 计算最小脉宽对应的比较值。
 *
 * @param timer_clock_hz 定时器计数时钟，单位 Hz。
 * @param prescaler 预分频值。
 * @param min_pulse_ns 最小脉宽约束，单位 ns。
 * @param period_ticks 自动重装值。
 * @param min_pulse_ticks 输出最小脉宽计数。
 * @return true 表示最小脉宽小于一个载波周期。
 *
 * 调用上下文：初始化和主机测试。
 * 失败行为：最小脉宽不小于自动重装值时返回 false。
 */
bool bsp_pwm_min_pulse_ticks(uint32_t timer_clock_hz,
                             uint32_t prescaler,
                             uint32_t min_pulse_ns,
                             uint32_t period_ticks,
                             uint32_t *min_pulse_ticks);

/**
 * @brief 判断时基状态是否满足 A4 闸门。
 *
 * @param status 时基状态。
 * @return true 表示中心对齐、载波、死区和最小脉宽全部满足要求。
 *
 * 调用上下文：自检、证据记录和主机测试。
 * 失败行为：状态为空或任一条件不满足时返回 false。
 */
bool bsp_pwm_timing_is_within_limits(const bsp_pwm_status_t *status);

/**
 * @brief 初始化两路 PWM 时基、死区和 ADC 注入触发。
 *
 * @return true 表示时基配置成功且输出保持禁用。
 *
 * 调用上下文：app_init，在安全输出和 DRV8323 安全初始化之后。
 * 失败行为：任一时基参数不可满足时返回 false，并调用 bsp_safe_outputs 保持安全态。
 */
bool bsp_pwm_init(void);

/**
 * @brief 读取指定轴的 PWM 时基状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出状态快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：自检、日志和证据记录。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_pwm_read_status(axis_id_t axis, bsp_pwm_status_t *status);

/**
 * @brief 回读指定轴的关键时基寄存器，用于闸门证据。
 *
 * @param axis 轴编号。
 * @param count_dir 输出计数方向寄存器值。
 * @param period 输出自动重装寄存器值。
 * @param prescaler 输出预分频寄存器值。
 * @param dead_time 输出死区寄存器原始值。
 * @return true 表示回读成功。
 *
 * 调用上下文：目标板无功率验证用例。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_pwm_read_registers(axis_id_t axis,
                            uint32_t *count_dir,
                            uint32_t *period,
                            uint32_t *prescaler,
                            uint32_t *dead_time);

/**
 * @brief 查询指定轴功率输出通道是否使能。
 *
 * @param axis 轴编号。
 * @return true 表示通道已使能。
 *
 * 调用上下文：自检和安全检查。
 * 失败行为：轴号非法时返回 true，保持保守策略。
 */
bool bsp_pwm_outputs_enabled(axis_id_t axis);

/**
 * @brief 在无功率条件下临时使能或关闭 PWM 测试波形。
 *
 * @param axis 轴编号。
 * @param enable true 表示使能测试波形，false 表示立即回到安全输出。
 * @return true 表示请求被接受。
 *
 * 调用上下文：仅限 A4 无功率验证用例，必须满足 PB12 为低且无母线功率。
 * 失败行为：PB12 未保持低电平、轴号非法或模块未初始化时拒绝使能并返回 false。
 */
bool bsp_pwm_test_enable_outputs(axis_id_t axis, bool enable);

#ifdef __cplusplus
}
#endif

#endif
