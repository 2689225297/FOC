/**
 * @file motor_observer.c
 * @brief A5 单轴观测器仲裁与交叉校验实现（EKF 主观测、SMO 回退、磁链监测）。
 *
 * 单周期顺序：
 *   1. SMO 与磁链观测器始终推进（SMO 既是回退候选也是交叉校验源）；
 *   2. ekf_enabled 时推进 EKF。EKF 的电压/电流输入按其自身估计角度转换到估计
 *      d-q 坐标系，与产品实现中电流环给出的 d-q 量一致；
 *   3. 仲裁：EKF 可用且转速达到门限时选 EKF；否则 SMO 可用时选 SMO；都不满足则
 *      置 angle_valid=false，由启动策略继续开环运行；
 *   4. 交叉校验：EKF 与 SMO 同时可用时比较角度，角度差持续超门限即锁存发散，
 *      angle_valid 保持 false，直到复位为止；
 *   5. 磁链监测：只在电频率不低于 10 倍截止频率时给出结论，否则置无效。
 *
 * 本文件为纯数值实现，不访问寄存器、不分配内存。
 */

#include "motor/motor_observer.h"
#include "motor/motor_transform.h"

#include <math.h>
#include <stddef.h>

/** @brief 机械转速换算系数：rad/s 电角速度到 rpm 的换算。 */
#define MOTOR_OBSERVER_RAD_S_TO_RPM 9.549296585513721f

/** @brief 磁链监测允许的相对偏差下限保护值。 */
#define MOTOR_OBSERVER_FLUX_EPSILON 1.0e-9f

/**
 * @brief 填充观测器默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_observer_default_config(motor_observer_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  (void)motor_ekf_default_config(&config->ekf);
  (void)motor_smo_default_config(&config->smo);
  (void)motor_flux_default_config(&config->flux);
  config->pole_pairs = 7.0f;
  config->flux_wb = 0.01f;
  config->min_close_speed_rpm = 60.0f;
  config->min_ekf_speed_rpm = 300.0f;
  config->min_smo_speed_rpm = 60.0f;
  config->cross_check_angle_rad = 0.35f;
  config->cross_check_hold_s = 0.05f;
  config->flux_error_ratio = 0.25f;
  /* 默认不启用任何角度源：必须由已归档的版本化标定记录显式放行。 */
  config->ekf_enabled = false;
  config->smo_enabled = false;
  config->flux_monitor_enabled = false;
  config->parameters_valid = false;

  return true;
}

/**
 * @brief 校验观测器配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_observer_validate_config(const motor_observer_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!config->parameters_valid)
  {
    /* 未辨识或未版本化的参数不得进入观测器配置。 */
    return false;
  }
  if (!config->ekf_enabled && !config->smo_enabled)
  {
    /* 没有任何可用角度源，观测器无法给出闭环角度。 */
    return false;
  }
  if (!motor_is_finite(config->pole_pairs) || (config->pole_pairs < 1.0f) ||
      (config->pole_pairs > 32.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->flux_wb) || (config->flux_wb <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->min_close_speed_rpm) || (config->min_close_speed_rpm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->min_ekf_speed_rpm) || (config->min_ekf_speed_rpm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->min_smo_speed_rpm) || (config->min_smo_speed_rpm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->cross_check_angle_rad) || (config->cross_check_angle_rad <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->cross_check_hold_s) || (config->cross_check_hold_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->flux_error_ratio) || (config->flux_error_ratio <= 0.0f))
  {
    return false;
  }
  if (!motor_smo_validate_config(&config->smo))
  {
    return false;
  }
  if (!motor_flux_validate_config(&config->flux))
  {
    return false;
  }
  if (config->ekf_enabled)
  {
    if (!motor_ekf_validate_config(&config->ekf))
    {
      return false;
    }
    if (!motor_is_finite(config->ekf.r_id) || (config->ekf.r_id < 0.1f) ||
        !motor_is_finite(config->ekf.r_iq) || (config->ekf.r_iq < 0.1f))
    {
      /* R/Q 未按实测电流噪声标定前不得启用 EKF 输出角度。 */
      return false;
    }
  }

  return true;
}

/**
 * @brief 复位观测器状态。
 *
 * @param state 输出状态。
 * @param config 观测器配置。
 * @param theta0_rad 初始电角度 rad。
 * @param omega0_rad_s 初始电角速度 rad/s。
 * @return 成功返回 true。
 */
bool motor_observer_reset(motor_observer_state_t *state,
                         const motor_observer_config_t *config,
                         float theta0_rad,
                         float omega0_rad_s)
{
  if ((state == NULL) || !motor_observer_validate_config(config))
  {
    return false;
  }
  if (!motor_is_finite(theta0_rad) || !motor_is_finite(omega0_rad_s))
  {
    return false;
  }

  if (!motor_smo_reset(&state->smo))
  {
    return false;
  }
  if (!motor_flux_reset(&state->flux))
  {
    return false;
  }
  if (config->ekf_enabled)
  {
    if (!motor_ekf_reset(&state->ekf, &config->ekf, theta0_rad, omega0_rad_s))
    {
      return false;
    }
  }

  state->active = MOTOR_OBSERVER_ACTIVE_NONE;
  state->theta_rad = motor_normalize_angle(theta0_rad);
  state->omega_e_rad_s = omega0_rad_s;
  state->speed_rpm = omega0_rad_s * MOTOR_OBSERVER_RAD_S_TO_RPM / config->pole_pairs;
  state->smo_speed_rpm = 0.0f;
  state->angle_error_rad = 0.0f;
  state->divergence_s = 0.0f;
  state->flux_mag_wb = 0.0f;
  state->flux_expected_wb = 0.0f;
  state->flux_error_ratio = 0.0f;
  state->reject_count = 0u;
  state->step_count = 0u;
  state->initialized = true;
  state->angle_valid = false;
  state->diverged = false;
  state->flux_monitor_ok = false;

  return true;
}

/**
 * @brief 查询当前角度是否可用于闭环。
 *
 * @param state 观测器状态。
 * @return true 表示角度可用且未锁存发散。
 */
bool motor_observer_is_angle_valid(const motor_observer_state_t *state)
{
  if (state == NULL)
  {
    return false;
  }

  return state->angle_valid;
}

/**
 * @brief 查询是否允许交班闭环。
 *
 * @param state 观测器状态。
 * @param config 观测器配置。
 * @return true 表示允许交班闭环。
 */
bool motor_observer_can_close_loop(const motor_observer_state_t *state,
                                  const motor_observer_config_t *config)
{
  if ((state == NULL) || (config == NULL))
  {
    return false;
  }
  if (!state->angle_valid || state->diverged)
  {
    return false;
  }
  if (fabsf(state->speed_rpm) < config->min_close_speed_rpm)
  {
    return false;
  }

  return true;
}

/**
 * @brief 推进一个控制周期的观测器仲裁。
 *
 * @param state 观测器状态。
 * @param config 观测器配置。
 * @param v_ab 本拍 α-β 电压 V。
 * @param i_ab 本拍 α-β 电流 A。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 */
bool motor_observer_step(motor_observer_state_t *state,
                        const motor_observer_config_t *config,
                        const motor_alpha_beta_t *v_ab,
                        const motor_alpha_beta_t *i_ab,
                        float dt_s)
{
  motor_dq_t idq_ekf;
  motor_dq_t vdq_ekf;
  float theta_ekf;
  float omega_ekf;
  float theta_smo;
  float omega_smo;
  float ekf_speed_rpm;
  float speed_reference_rpm;
  float omega_for_flux;
  float attenuation;
  bool smo_usable;
  bool ekf_usable;
  motor_observer_active_t wanted;

  if ((state == NULL) || !motor_observer_validate_config(config))
  {
    if (state != NULL)
    {
      state->angle_valid = false;
    }
    return false;
  }
  if ((v_ab == NULL) || (i_ab == NULL))
  {
    state->angle_valid = false;
    return false;
  }
  if (!motor_is_finite(v_ab->alpha) || !motor_is_finite(v_ab->beta) ||
      !motor_is_finite(i_ab->alpha) || !motor_is_finite(i_ab->beta))
  {
    state->angle_valid = false;
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f) || !state->initialized)
  {
    state->angle_valid = false;
    return false;
  }

  if (state->diverged)
  {
    /* 发散已锁存：保持角度无效，等待人工恢复后复位。 */
    state->active = MOTOR_OBSERVER_ACTIVE_NONE;
    state->angle_valid = false;
    ++state->step_count;
    return true;
  }

  if (!motor_smo_step(&state->smo, &config->smo, v_ab, i_ab, dt_s, &theta_smo, &omega_smo))
  {
    state->angle_valid = false;
    return false;
  }
  state->smo_speed_rpm = omega_smo * MOTOR_OBSERVER_RAD_S_TO_RPM / config->pole_pairs;

  ekf_speed_rpm = state->ekf.omega_est_rad_s * MOTOR_OBSERVER_RAD_S_TO_RPM / config->pole_pairs;
  if (config->ekf_enabled)
  {
    /* 按 EKF 自身估计角度把 α-β 量转换到其估计 d-q 坐标系。 */
    if (!motor_park(i_ab, state->ekf.theta_est_rad, &idq_ekf))
    {
      state->angle_valid = false;
      return false;
    }
    if (!motor_park(v_ab, state->ekf.theta_est_rad, &vdq_ekf))
    {
      state->angle_valid = false;
      return false;
    }
    if (!motor_ekf_step(&state->ekf, &config->ekf, &vdq_ekf, &idq_ekf, dt_s, &theta_ekf, &omega_ekf))
    {
      state->angle_valid = false;
      return false;
    }
    ekf_speed_rpm = omega_ekf * MOTOR_OBSERVER_RAD_S_TO_RPM / config->pole_pairs;
  }

  smo_usable = config->smo_enabled && state->smo.converged &&
               (fabsf(state->smo_speed_rpm) >= config->min_smo_speed_rpm);
  ekf_usable = config->ekf_enabled && (state->ekf.steps > 0u);

  speed_reference_rpm = smo_usable ? state->smo_speed_rpm : ekf_speed_rpm;

  /* 交叉校验：两个观测器同时可用时比较角度。 */
  if (ekf_usable && smo_usable)
  {
    state->angle_error_rad = fabsf(motor_wrap_pi(state->ekf.theta_est_rad - state->smo.theta_est_rad));
    if (state->angle_error_rad > config->cross_check_angle_rad)
    {
      state->divergence_s += dt_s;
    }
    else
    {
      state->divergence_s = 0.0f;
    }
    if (state->divergence_s >= config->cross_check_hold_s)
    {
      state->diverged = true;
      state->angle_valid = false;
      state->active = MOTOR_OBSERVER_ACTIVE_NONE;
      ++state->reject_count;
      ++state->step_count;
      return true;
    }
  }
  else
  {
    state->angle_error_rad = 0.0f;
    state->divergence_s = 0.0f;
  }

  if (ekf_usable && (fabsf(speed_reference_rpm) >= config->min_ekf_speed_rpm))
  {
    wanted = MOTOR_OBSERVER_ACTIVE_EKF;
  }
  else if (smo_usable)
  {
    wanted = MOTOR_OBSERVER_ACTIVE_SMO;
  }
  else
  {
    wanted = MOTOR_OBSERVER_ACTIVE_NONE;
  }

  /* 磁链监测：只在电频率不低于 10 倍截止频率时给出结论。 */
  omega_for_flux = (wanted == MOTOR_OBSERVER_ACTIVE_EKF) ? state->ekf.omega_est_rad_s
                                                         : state->smo.omega_est_rad_s;
  if (!motor_flux_step(&state->flux, &config->flux, v_ab, i_ab, omega_for_flux, dt_s))
  {
    state->angle_valid = false;
    return false;
  }
  state->flux_mag_wb = state->flux.flux_mag_wb;
  attenuation = motor_flux_magnitude_attenuation(&config->flux, fabsf(omega_for_flux));
  if (!motor_is_finite(attenuation) || (attenuation <= 0.0f))
  {
    attenuation = 0.0f;
  }
  state->flux_expected_wb = config->flux_wb * attenuation;
  if (state->flux_expected_wb > MOTOR_OBSERVER_FLUX_EPSILON)
  {
    state->flux_error_ratio = fabsf(state->flux_mag_wb - state->flux_expected_wb) /
                              state->flux_expected_wb;
  }
  else
  {
    state->flux_error_ratio = 0.0f;
  }
  state->flux_monitor_ok = config->flux_monitor_enabled &&
                           (fabsf(omega_for_flux) >=
                            (MOTOR_TWO_PI_F * config->flux.lpf_cutoff_hz * 10.0f)) &&
                           (state->flux_error_ratio <= config->flux_error_ratio);

  if (wanted == MOTOR_OBSERVER_ACTIVE_NONE)
  {
    /* 无可用角度源：保持角度无效，由启动策略继续开环运行。 */
    state->active = MOTOR_OBSERVER_ACTIVE_NONE;
    state->angle_valid = false;
    ++state->reject_count;
    ++state->step_count;
    return true;
  }

  if (wanted == MOTOR_OBSERVER_ACTIVE_EKF)
  {
    state->theta_rad = motor_normalize_angle(state->ekf.theta_est_rad);
    state->omega_e_rad_s = state->ekf.omega_est_rad_s;
    state->speed_rpm = ekf_speed_rpm;
  }
  else
  {
    state->theta_rad = motor_normalize_angle(state->smo.theta_est_rad);
    state->omega_e_rad_s = state->smo.omega_est_rad_s;
    state->speed_rpm = state->smo_speed_rpm;
  }

  state->active = wanted;
  state->angle_valid = true;
  ++state->step_count;

  return true;
}

/**
 * @brief 返回观测器名称。
 *
 * @param active 观测器编号。
 * @return 只读字符串；非法编号返回 "INVALID"。
 */
const char *motor_observer_active_name(motor_observer_active_t active)
{
  static const char *const names[MOTOR_OBSERVER_ACTIVE_COUNT] = {
      "NONE",
      "EKF",
      "SMO",
  };

  if ((uint32_t)active >= (uint32_t)MOTOR_OBSERVER_ACTIVE_COUNT)
  {
    return "INVALID";
  }

  return names[(uint32_t)active];
}
