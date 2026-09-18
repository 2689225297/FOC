/**
 * @file motor_hfi.h
 * @brief A5 产品化算法模块 HFI 方波注入与解调接口声明。
 *
 * HFI（高频注入）用于零低速凸极位置检测。本算法模块实现方波注入的极性生成、
 * 半周期同步解调、位置误差低通与跟踪积分，并把采样延迟折算为解调衰减系数，
 * 便于验证"解调正确性、延迟敏感性、噪声抑制-带宽权衡"三项数值行为。
 *
 * 选型约束：HFI 为实验性启动策略，置信度不足时必须回退开环 I/F，回退动作由
 * 启动策略模块（motor_startup）执行，不得在零低速下用 HFI 角度直接闭环。
 */
#ifndef MOTOR_HFI_H
#define MOTOR_HFI_H

#include "motor/motor_math.h"

/** @brief HFI 配置。 */
typedef struct
{
  float inject_voltage_v; /**< 注入方波电压幅值 V。 */
  float inject_freq_hz;   /**< 注入频率 Hz（全周期）。 */
  float lpf_alpha;        /**< 位置误差低通系数（0..1，每控制周期）。 */
  float tracking_gain;    /**< 位置跟踪增益 1/s。 */
  float sample_delay_s;   /**< 解调采样相对注入的延迟 s。 */
} motor_hfi_config_t;

/** @brief HFI 运行状态。 */
typedef struct
{
  float elapsed_s;         /**< 累计运行时间 s。 */
  float demod_accum;       /**< 当前半周期解调累积。 */
  float error_raw;         /**< 最近一次半周期解调误差。 */
  float error_filtered;    /**< 低通后的位置误差（sin(2Δθ) 量纲）。 */
  float angle_est_rad;     /**< 跟踪积分得到的位置估计 rad。 */
  int phase_index;         /**< 半周期序号，用于边界检测。 */
  int half_periods;        /**< 已完成半周期计数。 */
} motor_hfi_state_t;

/** @brief 填充默认配置：1kHz 注入、低通 0.05、跟踪增益 50 1/s。 */
bool motor_hfi_default_config(motor_hfi_config_t *config);

/** @brief 校验 HFI 配置合法性。 */
bool motor_hfi_validate_config(const motor_hfi_config_t *config);

/** @brief 复位 HFI 状态。 */
bool motor_hfi_reset(motor_hfi_state_t *state, const motor_hfi_config_t *config);

/** @brief 返回给定时刻的注入极性（+1 / -1），供上层构造注入电压。 */
int motor_hfi_inject_polarity(const motor_hfi_config_t *config, float elapsed_s);

/** @brief 返回给定注入相位下的解调参考极性（含采样延迟）。 */
int motor_hfi_demod_polarity(const motor_hfi_config_t *config, float elapsed_s);

/** @brief 凸极位置误差的理论映射 sin(2Δθ)。 */
float motor_hfi_expected_error(float theta_err_rad);

/** @brief 采样延迟导致的解调衰减系数（1 - 4·delay/T，可为负）。 */
float motor_hfi_delay_attenuation(const motor_hfi_config_t *config, float delay_s);

/** @brief 推进一个控制周期的解调与跟踪，输入为实测 q 轴电流样本。 */
bool motor_hfi_step(motor_hfi_state_t *state,
                   const motor_hfi_config_t *config,
                   float iq_sample_a,
                   float dt_s);

#endif
