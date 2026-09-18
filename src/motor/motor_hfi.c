/**
 * @file motor_hfi.c
 * @brief A5 产品化算法模块 HFI 方波注入解调实现（无硬件依赖）。
 *
 * 解调原理：注入极性翻转的 d 轴电压，q 轴电流响应与注入极性同相且幅值正比于
 * sin(2Δθ)。用含延迟的解调参考极性在半周期内做相关累积，再除以半周期长度得到
 * 位置误差；延迟 τ 使相关值按 (1 - 4τ/T) 线性衰减，τ = T/4 时完全失配。
 */

#include "motor/motor_hfi.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 填充 HFI 默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_hfi_default_config(motor_hfi_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->inject_voltage_v = 2.0f;
  config->inject_freq_hz = 1000.0f;
  config->lpf_alpha = 0.05f;
  config->tracking_gain = 50.0f;
  config->sample_delay_s = 0.0f;

  return true;
}

/**
 * @brief 校验 HFI 配置合法性。
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_hfi_validate_config(const motor_hfi_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->inject_voltage_v) || (config->inject_voltage_v <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->inject_freq_hz) || (config->inject_freq_hz <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->lpf_alpha) || (config->lpf_alpha <= 0.0f) || (config->lpf_alpha > 1.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->tracking_gain) || (config->tracking_gain < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->sample_delay_s) || (config->sample_delay_s < 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位 HFI 状态。
 * @param state HFI 状态。
 * @param config HFI 配置。
 * @return 成功返回 true。
 */
bool motor_hfi_reset(motor_hfi_state_t *state, const motor_hfi_config_t *config)
{
  if ((state == NULL) || !motor_hfi_validate_config(config))
  {
    return false;
  }

  state->elapsed_s = 0.0f;
  state->demod_accum = 0.0f;
  state->error_raw = 0.0f;
  state->error_filtered = 0.0f;
  state->angle_est_rad = 0.0f;
  state->phase_index = 0;
  state->half_periods = 0;

  return true;
}

/**
 * @brief 返回给定时刻的注入极性。
 * @param config HFI 配置。
 * @param elapsed_s 运行时间 s。
 * @return +1 或 -1；参数非法返回 0。
 */
int motor_hfi_inject_polarity(const motor_hfi_config_t *config, float elapsed_s)
{
  float half_period;
  float index;

  if (!motor_hfi_validate_config(config) || !motor_is_finite(elapsed_s))
  {
    return 0;
  }
  if (elapsed_s < 0.0f)
  {
    elapsed_s = 0.0f;
  }

  half_period = 1.0f / (2.0f * config->inject_freq_hz);
  index = elapsed_s / half_period;

  return (((int)index % 2) == 0) ? 1 : -1;
}

/**
 * @brief 返回含采样延迟的解调参考极性。
 * @param config HFI 配置。
 * @param elapsed_s 运行时间 s。
 * @return +1 或 -1；参数非法返回 0。
 */
int motor_hfi_demod_polarity(const motor_hfi_config_t *config, float elapsed_s)
{
  if (!motor_hfi_validate_config(config))
  {
    return 0;
  }

  return motor_hfi_inject_polarity(config, elapsed_s - config->sample_delay_s);
}

/**
 * @brief 凸极位置误差理论映射。
 * @param theta_err_rad 真实与估计位置之差 rad。
 * @return sin(2Δθ)。
 */
float motor_hfi_expected_error(float theta_err_rad)
{
  if (!motor_is_finite(theta_err_rad))
  {
    return 0.0f;
  }

  return sinf(2.0f * theta_err_rad);
}

/**
 * @brief 采样延迟导致的解调衰减系数。
 * @param config HFI 配置。
 * @param delay_s 延迟 s。
 * @return 1 - 4·delay/T（延迟按半周期折叠到界内）。
 */
float motor_hfi_delay_attenuation(const motor_hfi_config_t *config, float delay_s)
{
  float period;
  float ratio;

  if (!motor_hfi_validate_config(config) || !motor_is_finite(delay_s))
  {
    return 0.0f;
  }

  period = 1.0f / config->inject_freq_hz;
  ratio = delay_s / period;
  if (ratio < 0.0f)
  {
    ratio = -ratio;
  }
  /* 注入极性每半周期翻转，延迟按半周期折叠到 [0, 0.5] 区间。 */
  while (ratio > 0.5f)
  {
    ratio -= 0.5f;
  }

  return 1.0f - (4.0f * ratio);
}

/**
 * @brief 推进一个控制周期的解调与跟踪。
 * @param state HFI 状态。
 * @param config HFI 配置。
 * @param iq_sample_a 实测 q 轴电流样本 A。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_hfi_step(motor_hfi_state_t *state,
                   const motor_hfi_config_t *config,
                   float iq_sample_a,
                   float dt_s)
{
  float half_period;
  float demod_ref;
  int index;

  if ((state == NULL) || !motor_hfi_validate_config(config))
  {
    return false;
  }
  if (!motor_is_finite(iq_sample_a) || !motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }

  half_period = 1.0f / (2.0f * config->inject_freq_hz);
  state->elapsed_s += dt_s;

  demod_ref = (float)motor_hfi_demod_polarity(config, state->elapsed_s);
  state->demod_accum += demod_ref * iq_sample_a * dt_s;

  index = (int)(state->elapsed_s / half_period);
  if (index != state->phase_index)
  {
    state->phase_index = index;
    state->error_raw = state->demod_accum / half_period;
    state->demod_accum = 0.0f;
    state->half_periods++;
    state->error_filtered += (state->error_raw - state->error_filtered) * config->lpf_alpha;
  }

  state->angle_est_rad = motor_normalize_angle(
      state->angle_est_rad + (config->tracking_gain * state->error_filtered * dt_s));

  return true;
}
