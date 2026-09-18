/**
 * @file motor_if.c
 * @brief A5 产品化算法模块 I/F 强拖启动实现（无硬件依赖）。
 *
 * 行为约定：
 * 1. 预定位阶段电流幅值取 align_current_a，电角度固定为 0，频率为 0；
 * 2. 预定位结束后电流按 current_rate_a_s 上升到 align_current_a（以起点为
 *    0.25 倍电流开始爬升），频率按 freq_rate_hz_s 从 freq_start_hz 上升；
 * 3. 电角度按 omega*dt 连续累加，保证与频率一致；
 * 4. 频率达到 freq_end_hz 后置 ready_to_close，但本模块不执行任何闭环切换。
 */

#include "motor/motor_if.h"

#include <stddef.h>

/** @brief 加速阶段电流起始比例。 */
#define MOTOR_IF_CURRENT_START_RATIO 0.25f

/**
 * @brief 填充 I/F 默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_if_default_config(motor_if_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->align_time_s = 0.1f;
  config->align_current_a = 1.0f;
  config->current_rate_a_s = 5.0f;
  config->freq_start_hz = 2.0f;
  config->freq_rate_hz_s = 10.0f;
  config->freq_end_hz = 40.0f;

  return true;
}

/**
 * @brief 校验 I/F 配置合法性。
 * @param config 待校验配置。
 * @return 全部字段有限且取值合理返回 true。
 */
bool motor_if_validate_config(const motor_if_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->align_time_s) || (config->align_time_s < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->align_current_a) || (config->align_current_a <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->current_rate_a_s) || (config->current_rate_a_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->freq_start_hz) || (config->freq_start_hz < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->freq_rate_hz_s) || (config->freq_rate_hz_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->freq_end_hz) || (config->freq_end_hz <= config->freq_start_hz))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位 I/F 运行状态。
 * @param state 运行状态。
 * @param config I/F 配置。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_if_reset(motor_if_state_t *state, const motor_if_config_t *config)
{
  if ((state == NULL) || !motor_if_validate_config(config))
  {
    return false;
  }

  state->elapsed_s = 0.0f;
  state->align_phase = (config->align_time_s > 0.0f);
  state->current_ref_a = state->align_phase
                           ? config->align_current_a
                           : (config->align_current_a * MOTOR_IF_CURRENT_START_RATIO);
  state->freq_hz = state->align_phase ? 0.0f : config->freq_start_hz;
  state->omega_e_rad_s = MOTOR_TWO_PI_F * state->freq_hz;
  state->theta_e_rad = 0.0f;
  state->ready_to_close = false;

  return true;
}

/**
 * @brief 推进一个 I/F 控制周期。
 * @param state 运行状态。
 * @param config I/F 配置。
 * @param dt_s 控制周期 s。
 * @param idq_ref 输出的 d-q 电流指令（d=I、q=0）。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_if_step(motor_if_state_t *state,
                  const motor_if_config_t *config,
                  float dt_s,
                  motor_dq_t *idq_ref)
{
  if ((state == NULL) || (idq_ref == NULL) || !motor_if_validate_config(config))
  {
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }

  state->elapsed_s += dt_s;

  if (state->align_phase)
  {
    state->current_ref_a = config->align_current_a;
    state->freq_hz = 0.0f;
    if (state->elapsed_s >= config->align_time_s)
    {
      state->align_phase = false;
      state->current_ref_a = config->align_current_a * MOTOR_IF_CURRENT_START_RATIO;
      state->freq_hz = config->freq_start_hz;
    }
  }
  else
  {
    state->current_ref_a += config->current_rate_a_s * dt_s;
    if (state->current_ref_a > config->align_current_a)
    {
      state->current_ref_a = config->align_current_a;
    }

    state->freq_hz += config->freq_rate_hz_s * dt_s;
    if (state->freq_hz >= config->freq_end_hz)
    {
      state->freq_hz = config->freq_end_hz;
      state->ready_to_close = true;
    }
  }

  state->omega_e_rad_s = MOTOR_TWO_PI_F * state->freq_hz;
  state->theta_e_rad = motor_normalize_angle(state->theta_e_rad + (state->omega_e_rad_s * dt_s));

  idq_ref->d = state->current_ref_a;
  idq_ref->q = 0.0f;

  return true;
}
