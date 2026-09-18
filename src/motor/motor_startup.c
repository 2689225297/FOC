/**
 * @file motor_startup.c
 * @brief A5 单轴启动策略与观测器交班实现（V/F、I/F、HFI 及其回退）。
 *
 * 阶段顺序：
 *   IDLE -> ALIGN -> OPEN_LOOP_IF（或 OPEN_LOOP_VF / HFI）
 *        -> CLOSED_LOOP（观测器角度可用且开环频率达到交班门限）
 *   HFI 置信度持续不足 -> FALLBACK_IF，并把策略切换为开环 I/F 重新加速；
 *   交班后观测器角度连续失效超过超时门限 -> 退回开环 I/F；
 *   回退次数超过上限 -> start_failed。
 *
 * 本文件为纯数值实现，不访问寄存器、不分配内存。
 */

#include "motor/motor_startup.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 填充启动策略默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_startup_default_config(motor_startup_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->strategy = START_STRATEGY_OPEN_LOOP_IF;
  config->direction = MOTOR_DIRECTION_FORWARD;
  (void)motor_if_default_config(&config->if_config);
  (void)motor_hfi_default_config(&config->hfi_config);
  config->hfi_confidence_threshold = 0.35f;
  config->hfi_confidence_hold_s = 0.05f;
  config->closed_loop_min_freq_hz = 10.0f;
  config->observer_loss_timeout_s = 0.05f;
  config->vf_voltage_v = 2.0f;
  config->vf_freq_start_hz = 2.0f;
  config->vf_freq_rate_hz_s = 20.0f;
  config->vf_freq_end_hz = 40.0f;
  config->max_fallback_count = 2u;
  config->use_vf_mode = false;

  return true;
}

/**
 * @brief 校验启动策略配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_startup_validate_config(const motor_startup_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if ((config->strategy != START_STRATEGY_OPEN_LOOP_IF) &&
      (config->strategy != START_STRATEGY_HFI_EXPERIMENTAL))
  {
    return false;
  }
  if ((config->direction != MOTOR_DIRECTION_FORWARD) &&
      (config->direction != MOTOR_DIRECTION_REVERSE))
  {
    return false;
  }
  if (!motor_if_validate_config(&config->if_config))
  {
    return false;
  }
  if (!motor_hfi_validate_config(&config->hfi_config))
  {
    return false;
  }
  if (!motor_is_finite(config->hfi_confidence_threshold) ||
      (config->hfi_confidence_threshold <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->hfi_confidence_hold_s) || (config->hfi_confidence_hold_s < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->closed_loop_min_freq_hz) || (config->closed_loop_min_freq_hz < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->observer_loss_timeout_s) || (config->observer_loss_timeout_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->vf_voltage_v) || (config->vf_voltage_v <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->vf_freq_start_hz) || (config->vf_freq_start_hz < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->vf_freq_rate_hz_s) || (config->vf_freq_rate_hz_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->vf_freq_end_hz) || (config->vf_freq_end_hz <= config->vf_freq_start_hz))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位启动策略状态并进入预定位或 HFI 检测。
 *
 * @param state 输出状态。
 * @param config 启动策略配置。
 * @return 成功返回 true。
 */
bool motor_startup_reset(motor_startup_state_t *state, const motor_startup_config_t *config)
{
  if ((state == NULL) || !motor_startup_validate_config(config))
  {
    return false;
  }

  state->strategy = config->strategy;
  state->closed_loop = false;
  state->start_failed = false;
  state->hfi_fallback = false;
  state->vf_freq_hz = config->vf_freq_start_hz;
  state->vf_theta_rad = 0.0f;
  state->hfi_error = 0.0f;
  state->low_confidence_s = 0.0f;
  state->observer_loss_s = 0.0f;
  state->elapsed_s = 0.0f;
  state->fallback_count = 0u;
  state->step_count = 0u;

  if (!motor_if_reset(&state->if_state, &config->if_config))
  {
    return false;
  }
  if (!motor_hfi_reset(&state->hfi_state, &config->hfi_config))
  {
    return false;
  }

  state->phase = MOTOR_STARTUP_ALIGN;
  if (state->strategy == START_STRATEGY_HFI_EXPERIMENTAL)
  {
    state->phase = MOTOR_STARTUP_HFI;
  }
  else if (config->use_vf_mode)
  {
    state->phase = MOTOR_STARTUP_OPEN_LOOP_VF;
  }
  else
  {
    state->phase = MOTOR_STARTUP_ALIGN;
  }

  return true;
}

/**
 * @brief 返回旋转方向系数。
 *
 * @param direction 目标方向。
 * @return 正转为 +1，反转为 -1。
 *
 * 调用上下文：开环角度与频率输出。
 * 失败行为：无失败路径。
 */
static float motor_startup_direction_sign(motor_direction_t direction)
{
  return (direction == MOTOR_DIRECTION_REVERSE) ? -1.0f : 1.0f;
}

/**
 * @brief 把 HFI 阶段切换到开环 I/F 回退阶段。
 *
 * @param state 启动策略状态。
 * @param config 启动策略配置。
 * @return 无返回值。
 *
 * 调用上下文：HFI 置信度不足或闭环观测器角度失效。
 * 失败行为：I/F 复位失败时置 start_failed。
 */
static void motor_startup_fallback_to_if(motor_startup_state_t *state,
                                        const motor_startup_config_t *config)
{
  ++state->fallback_count;
  state->hfi_fallback = true;
  state->closed_loop = false;
  state->low_confidence_s = 0.0f;
  state->observer_loss_s = 0.0f;
  state->strategy = START_STRATEGY_OPEN_LOOP_IF;
  state->phase = MOTOR_STARTUP_OPEN_LOOP_IF;
  if (!motor_if_reset(&state->if_state, &config->if_config))
  {
    state->start_failed = true;
  }
  if (state->fallback_count > config->max_fallback_count)
  {
    state->start_failed = true;
  }
}

/**
 * @brief 推进一个控制周期的启动策略。
 *
 * @param state 启动策略状态。
 * @param config 启动策略配置。
 * @param iq_sample_a 实测 q 轴电流样本 A。
 * @param observer_ready 观测器是否允许交班闭环。
 * @param observer_valid 观测器角度当前是否有效。
 * @param dt_s 控制周期 s。
 * @param output 单周期输出。
 * @return 成功返回 true。
 */
bool motor_startup_step(motor_startup_state_t *state,
                       const motor_startup_config_t *config,
                       float iq_sample_a,
                       bool observer_ready,
                       bool observer_valid,
                       float dt_s,
                       motor_startup_output_t *output)
{
  float direction_sign;

  if ((state == NULL) || (output == NULL) || !motor_startup_validate_config(config))
  {
    return false;
  }
  if (!motor_is_finite(iq_sample_a) || !motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }

  direction_sign = motor_startup_direction_sign(config->direction);
  state->elapsed_s += dt_s;
  ++state->step_count;
  state->hfi_fallback = false;

  output->idq_ref.d = 0.0f;
  output->idq_ref.q = 0.0f;
  output->vdq_ref.d = 0.0f;
  output->vdq_ref.q = 0.0f;
  output->theta_rad = 0.0f;
  output->omega_e_rad_s = 0.0f;
  output->hfi_inject_voltage_v = 0.0f;
  output->mode = MOTOR_MODE_STOP;
  output->closed_loop = state->closed_loop;
  output->hfi_fallback = false;
  output->start_failed = state->start_failed;

  if (state->start_failed)
  {
    return true;
  }

  if (state->closed_loop)
  {
    /* 交班后角度由观测器提供，本模块只负责失效超时判据与回退。 */
    if (observer_valid)
    {
      state->observer_loss_s = 0.0f;
    }
    else
    {
      state->observer_loss_s += dt_s;
      if (state->observer_loss_s >= config->observer_loss_timeout_s)
      {
        motor_startup_fallback_to_if(state, config);
        output->hfi_fallback = state->hfi_fallback;
        output->start_failed = state->start_failed;
        return true;
      }
    }
    output->closed_loop = true;
    output->mode = MOTOR_MODE_CURRENT_IDIQ;
    return true;
  }

  switch (state->phase)
  {
    case MOTOR_STARTUP_ALIGN:
    case MOTOR_STARTUP_OPEN_LOOP_IF:
    case MOTOR_STARTUP_FALLBACK_IF:
    {
      motor_dq_t idq_ref;

      if (!motor_if_step(&state->if_state, &config->if_config, dt_s, &idq_ref))
      {
        return false;
      }
      output->idq_ref = idq_ref;
      output->theta_rad = motor_normalize_angle(direction_sign * state->if_state.theta_e_rad);
      output->omega_e_rad_s = direction_sign * state->if_state.omega_e_rad_s;
      output->mode = MOTOR_MODE_OPEN_LOOP_IF;
      state->phase = state->if_state.align_phase ? MOTOR_STARTUP_ALIGN : MOTOR_STARTUP_OPEN_LOOP_IF;
      break;
    }
    case MOTOR_STARTUP_OPEN_LOOP_VF:
    {
      float ratio;

      state->vf_freq_hz += config->vf_freq_rate_hz_s * dt_s;
      if (state->vf_freq_hz > config->vf_freq_end_hz)
      {
        state->vf_freq_hz = config->vf_freq_end_hz;
      }
      ratio = state->vf_freq_hz / config->vf_freq_end_hz;
      state->vf_theta_rad = motor_normalize_angle(
          state->vf_theta_rad + (direction_sign * MOTOR_TWO_PI_F * state->vf_freq_hz * dt_s));
      output->vdq_ref.d = 0.0f;
      output->vdq_ref.q = config->vf_voltage_v * ratio;
      output->theta_rad = state->vf_theta_rad;
      output->omega_e_rad_s = direction_sign * MOTOR_TWO_PI_F * state->vf_freq_hz;
      output->mode = MOTOR_MODE_OPEN_LOOP_VF;
      break;
    }
    case MOTOR_STARTUP_HFI:
    {
      if (!motor_hfi_step(&state->hfi_state, &config->hfi_config, iq_sample_a, dt_s))
      {
        return false;
      }
      /* HFI 位置误差越大越不可信；误差持续超门限即自动回退开环 I/F。 */
      state->hfi_error = fabsf(state->hfi_state.error_filtered);
      output->theta_rad = motor_normalize_angle(direction_sign * state->hfi_state.angle_est_rad);
      output->omega_e_rad_s = 0.0f;
      output->hfi_inject_voltage_v =
          ((float)motor_hfi_inject_polarity(&config->hfi_config, state->hfi_state.elapsed_s)) *
          config->hfi_config.inject_voltage_v;
      output->mode = MOTOR_MODE_OPEN_LOOP_IF;

      if (state->hfi_error > config->hfi_confidence_threshold)
      {
        state->low_confidence_s += dt_s;
      }
      else
      {
        state->low_confidence_s = 0.0f;
      }
      if (state->low_confidence_s >= config->hfi_confidence_hold_s)
      {
        /* HFI 不可靠：立即回退开环 I/F。 */
        motor_startup_fallback_to_if(state, config);
        output->theta_rad = motor_normalize_angle(direction_sign * state->if_state.theta_e_rad);
        output->omega_e_rad_s = direction_sign * state->if_state.omega_e_rad_s;
        output->hfi_inject_voltage_v = 0.0f;
        output->mode = MOTOR_MODE_OPEN_LOOP_IF;
        output->idq_ref.d = state->if_state.current_ref_a;
        output->idq_ref.q = 0.0f;
        output->hfi_fallback = state->hfi_fallback;
        output->start_failed = state->start_failed;
        return true;
      }
      break;
    }
    case MOTOR_STARTUP_CLOSED_LOOP:
    case MOTOR_STARTUP_IDLE:
    case MOTOR_STARTUP_COUNT:
    default:
      return false;
  }

  /* 交班判据：观测器角度可用且开环频率已达到交班门限。 */
  if (observer_ready && (fabsf(output->omega_e_rad_s) >= (MOTOR_TWO_PI_F * config->closed_loop_min_freq_hz)))
  {
    state->closed_loop = true;
    state->observer_loss_s = 0.0f;
    output->closed_loop = true;
    output->mode = MOTOR_MODE_CURRENT_IDIQ;
    output->hfi_inject_voltage_v = 0.0f;
  }

  output->hfi_fallback = state->hfi_fallback;
  output->start_failed = state->start_failed;

  return true;
}

/**
 * @brief 返回启动策略阶段名称。
 *
 * @param phase 阶段编号。
 * @return 只读字符串；非法阶段返回 "INVALID"。
 */
const char *motor_startup_phase_name(motor_startup_phase_t phase)
{
  static const char *const names[MOTOR_STARTUP_COUNT] = {
      "IDLE",
      "ALIGN",
      "OPEN_LOOP_IF",
      "OPEN_LOOP_VF",
      "HFI",
      "CLOSED_LOOP",
      "FALLBACK_IF",
  };

  if ((uint32_t)phase >= (uint32_t)MOTOR_STARTUP_COUNT)
  {
    return "INVALID";
  }

  return names[(uint32_t)phase];
}
