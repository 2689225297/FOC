/**
 * @file motor_flux.h
 * @brief A5 产品化算法模块定子磁链观测器接口声明（电压模型 + 低通漂移抑制）。
 *
 * 采用 α-β 轴电压模型积分，配合一阶低通抑制纯积分带来的直流漂移。该结构在
 * 低速段存在幅值衰减与相位滞后，产品级实现只用于中高速磁链幅值监测与参数
 * 辨识辅助，不独立承担角度输出；电频率低于约 10 倍截止频率时不得用于监测判据。
 */
#ifndef MOTOR_FLUX_H
#define MOTOR_FLUX_H

#include "motor/motor_math.h"

/** @brief 磁链观测器配置。 */
typedef struct
{
  float rs_ohm;          /**< 定子电阻估计值 Ω。 */
  float lpf_cutoff_hz;   /**< 低通积分截止频率 Hz。 */
} motor_flux_config_t;

/** @brief 磁链观测器状态。 */
typedef struct
{
  motor_alpha_beta_t flux_ab; /**< 估计 α-β 定子磁链 Wb。 */
  float flux_mag_wb;         /**< 磁链幅值 Wb。 */
  float theta_est_rad;       /**< 由磁链矢量得到的角度 rad（未做凸极修正）。 */
} motor_flux_state_t;

/** @brief 填充默认配置：Rs=0.05Ω、截止频率 5Hz。 */
bool motor_flux_default_config(motor_flux_config_t *config);

/** @brief 校验配置合法性。 */
bool motor_flux_validate_config(const motor_flux_config_t *config);

/** @brief 复位磁链观测器状态。 */
bool motor_flux_reset(motor_flux_state_t *state);

/** @brief 推进一个控制周期，输入 α-β 电压、α-β 电流与电角速度。 */
bool motor_flux_step(motor_flux_state_t *state,
                    const motor_flux_config_t *config,
                    const motor_alpha_beta_t *v_ab,
                    const motor_alpha_beta_t *i_ab,
                    float omega_e_rad_s,
                    float dt_s);

/** @brief 计算低通积分器在给定电角速度下的幅值衰减系数。 */
float motor_flux_magnitude_attenuation(const motor_flux_config_t *config, float omega_e_rad_s);

/** @brief 计算低通积分器在给定电角速度下的相位滞后 rad（正值表示滞后）。 */
float motor_flux_phase_lag(const motor_flux_config_t *config, float omega_e_rad_s);

#endif
