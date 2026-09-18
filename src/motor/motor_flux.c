/**
 * @file motor_flux.c
 * @brief A5 产品化算法模块定子磁链观测器实现（无硬件依赖）。
 *
 * 离散化：ψ ← ψ + dt·(v - Rs·i - ωc·ψ)，其中 ωc = 2π·f_cutoff。
 * 该式等价于对反电动势做一阶低通后积分，稳态幅值衰减 ω/√(ω²+ωc²)、
 * 相位滞后 atan(ωc/ω)，便于与解析式对照验证。
 */

#include "motor/motor_flux.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 填充磁链观测器默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_flux_default_config(motor_flux_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->rs_ohm = 0.05f;
  config->lpf_cutoff_hz = 5.0f;

  return true;
}

/**
 * @brief 校验磁链观测器配置。
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_flux_validate_config(const motor_flux_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->rs_ohm) || (config->rs_ohm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->lpf_cutoff_hz) || (config->lpf_cutoff_hz <= 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位磁链观测器状态。
 * @param state 观测器状态。
 * @return 状态指针非空返回 true。
 */
bool motor_flux_reset(motor_flux_state_t *state)
{
  if (state == NULL)
  {
    return false;
  }

  state->flux_ab.alpha = 0.0f;
  state->flux_ab.beta = 0.0f;
  state->flux_mag_wb = 0.0f;
  state->theta_est_rad = 0.0f;

  return true;
}

/**
 * @brief 推进一个控制周期。
 * @param state 观测器状态。
 * @param config 观测器配置。
 * @param v_ab α-β 电压 V。
 * @param i_ab α-β 电流 A。
 * @param omega_e_rad_s 电角速度 rad/s（仅用于输出参考，不参与积分）。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_flux_step(motor_flux_state_t *state,
                    const motor_flux_config_t *config,
                    const motor_alpha_beta_t *v_ab,
                    const motor_alpha_beta_t *i_ab,
                    float omega_e_rad_s,
                    float dt_s)
{
  float wc;
  float e_alpha;
  float e_beta;
  float flux_alpha;
  float flux_beta;

  (void)omega_e_rad_s;

  if ((state == NULL) || !motor_flux_validate_config(config))
  {
    return false;
  }
  if ((v_ab == NULL) || (i_ab == NULL))
  {
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(v_ab->alpha) || !motor_is_finite(v_ab->beta) ||
      !motor_is_finite(i_ab->alpha) || !motor_is_finite(i_ab->beta))
  {
    return false;
  }

  wc = MOTOR_TWO_PI_F * config->lpf_cutoff_hz;
  e_alpha = v_ab->alpha - (config->rs_ohm * i_ab->alpha);
  e_beta = v_ab->beta - (config->rs_ohm * i_ab->beta);

  flux_alpha = state->flux_ab.alpha + (dt_s * (e_alpha - (wc * state->flux_ab.alpha)));
  flux_beta = state->flux_ab.beta + (dt_s * (e_beta - (wc * state->flux_ab.beta)));

  state->flux_ab.alpha = flux_alpha;
  state->flux_ab.beta = flux_beta;
  state->flux_mag_wb = sqrtf((flux_alpha * flux_alpha) + (flux_beta * flux_beta));
  if (state->flux_mag_wb > 1.0e-9f)
  {
    state->theta_est_rad = motor_normalize_angle(atan2f(flux_beta, flux_alpha));
  }

  return true;
}

/**
 * @brief 计算低通积分器幅值衰减系数。
 * @param config 观测器配置。
 * @param omega_e_rad_s 电角速度 rad/s。
 * @return ω/√(ω²+ωc²)；ω=0 时返回 0。
 */
float motor_flux_magnitude_attenuation(const motor_flux_config_t *config, float omega_e_rad_s)
{
  float wc;

  if (!motor_flux_validate_config(config) || !motor_is_finite(omega_e_rad_s))
  {
    return 0.0f;
  }

  wc = MOTOR_TWO_PI_F * config->lpf_cutoff_hz;
  if (omega_e_rad_s <= 0.0f)
  {
    return 0.0f;
  }

  return omega_e_rad_s / sqrtf((omega_e_rad_s * omega_e_rad_s) + (wc * wc));
}

/**
 * @brief 计算低通积分器相位滞后。
 * @param config 观测器配置。
 * @param omega_e_rad_s 电角速度 rad/s。
 * @return 滞后角 rad；ω=0 时返回 π/2。
 */
float motor_flux_phase_lag(const motor_flux_config_t *config, float omega_e_rad_s)
{
  float wc;

  if (!motor_flux_validate_config(config) || !motor_is_finite(omega_e_rad_s))
  {
    return 0.0f;
  }

  wc = MOTOR_TWO_PI_F * config->lpf_cutoff_hz;
  if (omega_e_rad_s <= 0.0f)
  {
    return MOTOR_PI_F * 0.5f;
  }

  return atanf(wc / omega_e_rad_s);
}
