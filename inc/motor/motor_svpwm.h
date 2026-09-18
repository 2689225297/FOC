/**
 * @file motor_svpwm.h
 * @brief A5 产品化算法模块 SVPWM 调制接口声明（含死区与母线调制限制）。
 *
 * 采用三相参考电压加零序注入的等效 SVPWM 实现，线性区最大输出矢量幅值为
 * Udc/√3。调制结果同时受母线电压限幅与死区/最小脉宽约束，所有约束均以
 * 标志位回传，便于主机测试逐项验证。
 */
#ifndef MOTOR_SVPWM_H
#define MOTOR_SVPWM_H

#include "motor/motor_math.h"

/** @brief SVPWM 配置参数。 */
typedef struct
{
  float carrier_hz;    /**< 载波频率 Hz（中心对齐计数）。 */
  float bus_voltage_v; /**< 母线电压 V。 */
  float dead_time_s;   /**< 单桥臂死区时间 s。 */
  float min_pulse_s;   /**< 允许的最小有效脉宽 s（不含死区）。 */
} motor_svpwm_config_t;

/** @brief SVPWM 单周期求解结果。 */
typedef struct
{
  float duty[3];              /**< 三相上桥臂占空比，范围 0..1。 */
  float modulation_index;     /**< 归一化调制比，1.0 为线性区边界。 */
  float v_max_linear;         /**< 线性区最大 α-β 矢量幅值 V。 */
  bool limited;               /**< 是否因母线电压约束触发限幅。 */
  bool dead_time_clamped;     /**< 是否因死区/最小脉宽约束钳制占空比。 */
  bool near_full_modulation;  /**< 是否已经接近线性区边界。 */
} motor_svpwm_result_t;

/** @brief 填充默认配置：20kHz、24V 母线、500ns 死区、1000ns 最小脉宽。 */
bool motor_svpwm_default_config(motor_svpwm_config_t *config);

/** @brief 校验配置合法性，非法返回 false。 */
bool motor_svpwm_validate_config(const motor_svpwm_config_t *config);

/** @brief 返回线性区最大 α-β 电压矢量幅值 Udc/√3。 */
float motor_svpwm_max_linear_voltage(const motor_svpwm_config_t *config);

/** @brief 返回由死区与最小脉宽决定的最小占空比份额。 */
float motor_svpwm_min_duty(const motor_svpwm_config_t *config);

/** @brief 求解一周期三相占空比，输入为 α-β 参考电压。 */
bool motor_svpwm_solve(const motor_svpwm_config_t *config,
                      const motor_alpha_beta_t *v_ref,
                      motor_svpwm_result_t *result);

/** @brief 返回参考电压所在扇区（1..6），零矢量返回 0。 */
int motor_svpwm_sector(const motor_alpha_beta_t *v_ref);

/** @brief 由三相占空比反解 α-β 参考电压，用于调制一致性校验。 */
bool motor_svpwm_duty_to_alpha_beta(const motor_svpwm_config_t *config,
                                   const float duty[3],
                                   motor_alpha_beta_t *v_out);

#endif
