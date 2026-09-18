/**
 * @file motor_control.c
 * @brief A5 单轴电流环控制路径实现（Clarke/Park、Id/Iq PI、解耦前馈、限幅、SVPWM）。
 *
 * 单周期顺序：
 *   1. Clarke/Park 得到 d-q 电流；
 *   2. 指令误差经 d-q 两个 PI（条件积分抗饱和）得到反馈电压；
 *   3. 按 PMSM 电压方程叠加解耦前馈；
 *   4. 电压矢量整体限幅（不超过配置限幅与母线线性区 Udc/√3）；
 *   5. 反 Park 得到 α-β 电压，交 SVPWM 求三相占空比；
 *   6. 任一环节非有限、参数未标定或处于安全停机态时，拒绝输出并保持占空比不变。
 *
 * 本文件为纯数值实现，不访问寄存器、不分配内存，可直接在主机单元测试中运行。
 */

#include "motor/motor_control.h"
#include "motor/motor_transform.h"

#include <math.h>
#include <stddef.h>

/** @brief 安全停机时下发的零电压中点占空比。 */
#define MOTOR_CONTROL_NEUTRAL_DUTY 0.5f

/**
 * @brief 填充电流环默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_control_default_config(motor_control_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  /* 默认电机参数沿用路径 C 试验条件，但 parameters_valid 保持 false，禁止直接使用。 */
  config->motor.rs_ohm = 0.05f;
  config->motor.ld_h = 0.0002f;
  config->motor.lq_h = 0.0005f;
  config->motor.flux_wb = 0.01f;
  config->kp_d = 0.0f;
  config->ki_d = 0.0f;
  config->kp_q = 0.0f;
  config->ki_q = 0.0f;
  config->voltage_limit_v = 0.0f;
  config->current_limit_a = 0.0f;
  config->enable_decoupling = true;
  config->parameters_valid = false;
  (void)motor_svpwm_default_config(&config->svpwm);

  return true;
}

/**
 * @brief 校验电流环配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_control_validate_config(const motor_control_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!config->parameters_valid)
  {
    /* 未标定或未版本化参数不得进入控制路径。 */
    return false;
  }
  if (!motor_is_finite(config->motor.rs_ohm) || (config->motor.rs_ohm <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->motor.ld_h) || (config->motor.ld_h <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->motor.lq_h) || (config->motor.lq_h <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->motor.flux_wb) || (config->motor.flux_wb <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->kp_d) || (config->kp_d < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->ki_d) || (config->ki_d < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->kp_q) || (config->kp_q < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->ki_q) || (config->ki_q < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->voltage_limit_v) || (config->voltage_limit_v < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->current_limit_a) || (config->current_limit_a <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->svpwm.bus_voltage_v) || (config->svpwm.bus_voltage_v <= 0.0f))
  {
    return false;
  }
  if (!motor_svpwm_validate_config(&config->svpwm))
  {
    return false;
  }

  return true;
}

/**
 * @brief 返回本配置下生效的电压矢量限幅值。
 *
 * @param config 电流环配置。
 * @return 限幅值 V；配置非法返回 0。
 */
float motor_control_voltage_limit(const motor_control_config_t *config)
{
  float linear_limit;

  if (config == NULL)
  {
    return 0.0f;
  }

  linear_limit = motor_svpwm_max_linear_voltage(&config->svpwm);
  if (!motor_is_finite(linear_limit) || (linear_limit < 0.0f))
  {
    return 0.0f;
  }
  if ((config->voltage_limit_v > 0.0f) && (config->voltage_limit_v < linear_limit))
  {
    return config->voltage_limit_v;
  }

  return linear_limit;
}

/**
 * @brief 复位运行状态（保留 PI 增益）。
 *
 * @param state 电流环状态。
 * @return 无返回值。
 */
void motor_control_reset(motor_control_state_t *state)
{
  uint32_t index;

  if (state == NULL)
  {
    return;
  }

  motor_pi_reset(&state->pi_d);
  motor_pi_reset(&state->pi_q);
  state->i_ab.alpha = 0.0f;
  state->i_ab.beta = 0.0f;
  state->idq.d = 0.0f;
  state->idq.q = 0.0f;
  state->idq_ref.d = 0.0f;
  state->idq_ref.q = 0.0f;
  state->vdq_ff.d = 0.0f;
  state->vdq_ff.q = 0.0f;
  /* 复位即关闭注入，禁止残留注入电压在下次启动时被误下发。 */
  state->vd_inject_v = 0.0f;
  state->vdq.d = 0.0f;
  state->vdq.q = 0.0f;
  state->v_ab.alpha = 0.0f;
  state->v_ab.beta = 0.0f;
  state->step_count = 0u;
  state->limit_count = 0u;
  state->reference_limited = false;
  state->limited = false;
  state->stopped = true;

  for (index = 0u; index < 3u; ++index)
  {
    state->duty[index] = MOTOR_CONTROL_NEUTRAL_DUTY;
  }
  state->modulation.duty[0] = MOTOR_CONTROL_NEUTRAL_DUTY;
  state->modulation.duty[1] = MOTOR_CONTROL_NEUTRAL_DUTY;
  state->modulation.duty[2] = MOTOR_CONTROL_NEUTRAL_DUTY;
  state->modulation.modulation_index = 0.0f;
  state->modulation.v_max_linear = 0.0f;
  state->modulation.limited = false;
  state->modulation.dead_time_clamped = false;
  state->modulation.near_full_modulation = false;
}

/**
 * @brief 初始化电流环状态与 PI 调节器。
 *
 * @param state 输出状态。
 * @param config 电流环配置。
 * @return 成功返回 true。
 */
bool motor_control_init(motor_control_state_t *state, const motor_control_config_t *config)
{
  float v_limit;

  if ((state == NULL) || !motor_control_validate_config(config))
  {
    return false;
  }

  v_limit = motor_control_voltage_limit(config);
  if (!motor_is_finite(v_limit) || (v_limit <= 0.0f))
  {
    return false;
  }

  motor_pi_init(&state->pi_d, config->kp_d, config->ki_d, -v_limit, v_limit);
  motor_pi_init(&state->pi_q, config->kp_q, config->ki_q, -v_limit, v_limit);
  motor_pi_set_integral_limit(&state->pi_d, v_limit);
  motor_pi_set_integral_limit(&state->pi_q, v_limit);
  motor_control_reset(state);
  state->v_limit_v = v_limit;

  return true;
}

/**
 * @brief 设置 d-q 电流指令并解除安全停机标志。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param idq_ref 指令电流 A。
 * @return 成功返回 true。
 */
bool motor_control_set_current_reference(motor_control_state_t *state,
                                        const motor_control_config_t *config,
                                        const motor_dq_t *idq_ref)
{
  float magnitude;
  float scale;

  if ((state == NULL) || !motor_control_validate_config(config) || (idq_ref == NULL))
  {
    return false;
  }
  if (!motor_is_finite(idq_ref->d) || !motor_is_finite(idq_ref->q))
  {
    return false;
  }

  magnitude = sqrtf((idq_ref->d * idq_ref->d) + (idq_ref->q * idq_ref->q));
  state->reference_limited = false;
  if (magnitude > config->current_limit_a)
  {
    scale = config->current_limit_a / magnitude;
    state->idq_ref.d = idq_ref->d * scale;
    state->idq_ref.q = idq_ref->q * scale;
    state->reference_limited = true;
  }
  else
  {
    state->idq_ref = *idq_ref;
  }

  /* 接受电流指令即表示控制路径转入运行态，调用方必须已完成停机条件检查。 */
  state->stopped = false;

  return true;
}

/**
 * @brief 设置 d 轴高频注入电压（与电流环 PI 输出叠加）。
 *
 * @param state 电流环状态。
 * @param vd_inject_v 注入电压 V；0 表示关闭注入。
 * @return 成功返回 true。
 */
bool motor_control_set_voltage_injection(motor_control_state_t *state, float vd_inject_v)
{
  if (state == NULL)
  {
    return false;
  }
  if (!motor_is_finite(vd_inject_v))
  {
    return false;
  }

  /* 注入值允许为负（方波极性），但只作为叠加项，最终统一受电压矢量限幅约束。 */
  state->vd_inject_v = vd_inject_v;

  return true;
}

/**
 * @brief 进入安全停机：清零积分与限幅标志，占空比置零电压中点。
 *
 * @param state 电流环状态。
 * @return true 表示已进入安全停机。
 */
bool motor_control_safe_stop(motor_control_state_t *state)
{
  if (state == NULL)
  {
    return false;
  }

  motor_control_reset(state);
  state->stopped = true;

  return true;
}

/**
 * @brief 查询是否处于安全停机态。
 *
 * @param state 电流环状态。
 * @return true 表示已安全停机。
 */
bool motor_control_is_stopped(const motor_control_state_t *state)
{
  if (state == NULL)
  {
    return true;
  }

  return state->stopped;
}

/**
 * @brief 检查当前三相占空比是否为可下发的合法值。
 *
 * @param state 电流环状态。
 * @return true 表示三路占空比合法。
 */
bool motor_control_duty_is_safe(const motor_control_state_t *state)
{
  uint32_t index;

  if (state == NULL)
  {
    return false;
  }

  for (index = 0u; index < 3u; ++index)
  {
    if (!motor_is_finite(state->duty[index]))
    {
      return false;
    }
    if ((state->duty[index] < 0.0f) || (state->duty[index] > 1.0f))
    {
      return false;
    }
  }

  return true;
}

/**
 * @brief 执行一拍电流环运算并更新占空比。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param i_abc 三相实测电流 A。
 * @param theta_rad 本拍电角度 rad。
 * @param omega_e_rad_s 本拍电角速度 rad/s。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 */
bool motor_control_step(motor_control_state_t *state,
                       const motor_control_config_t *config,
                       const motor_abc_t *i_abc,
                       float theta_rad,
                       float omega_e_rad_s,
                       float dt_s)
{
  motor_alpha_beta_t i_ab;
  motor_dq_t error;
  motor_dq_t command;
  float v_limit;
  float magnitude;
  float scale;

  if ((state == NULL) || !motor_control_validate_config(config) || (i_abc == NULL))
  {
    return false;
  }
  if (!motor_is_finite(theta_rad) || !motor_is_finite(omega_e_rad_s))
  {
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(i_abc->a) || !motor_is_finite(i_abc->b) || !motor_is_finite(i_abc->c))
  {
    return false;
  }
  if (state->stopped)
  {
    /* 安全停机态拒绝恢复输出，必须由 motor_control_set_current_reference 显式解除。 */
    return false;
  }

  if (!motor_clarke(i_abc, &i_ab))
  {
    return false;
  }
  if (!motor_park(&i_ab, theta_rad, &state->idq))
  {
    return false;
  }
  state->i_ab = i_ab;
  if (!motor_is_finite(state->idq.d) || !motor_is_finite(state->idq.q))
  {
    return false;
  }

  error.d = state->idq_ref.d - state->idq.d;
  error.q = state->idq_ref.q - state->idq.q;
  command.d = motor_pi_step(&state->pi_d, error.d, dt_s);
  command.q = motor_pi_step(&state->pi_q, error.q, dt_s);

  state->vdq_ff.d = 0.0f;
  state->vdq_ff.q = 0.0f;
  if (config->enable_decoupling)
  {
    motor_decoupling_feedforward(&config->motor, &state->idq, omega_e_rad_s, &state->vdq_ff);
    if (!motor_is_finite(state->vdq_ff.d) || !motor_is_finite(state->vdq_ff.q))
    {
      return false;
    }
    command.d += state->vdq_ff.d;
    command.q += state->vdq_ff.q;
  }

  /* 高频注入叠加在 d 轴，与 PI、前馈一起接受同一电压矢量限幅约束。 */
  command.d += state->vd_inject_v;

  v_limit = motor_control_voltage_limit(config);
  state->v_limit_v = v_limit;
  state->limited = false;
  if (!motor_is_finite(command.d) || !motor_is_finite(command.q) || (v_limit <= 0.0f))
  {
    return false;
  }

  magnitude = sqrtf((command.d * command.d) + (command.q * command.q));
  if (!motor_is_finite(magnitude))
  {
    return false;
  }
  if (magnitude > v_limit)
  {
    /* 电压矢量整体等比例限幅，限幅后不得再放大。 */
    scale = v_limit / magnitude;
    command.d *= scale;
    command.q *= scale;
    state->limited = true;
    ++state->limit_count;
  }
  state->vdq = command;

  if (!motor_inverse_park(&state->vdq, theta_rad, &state->v_ab))
  {
    return false;
  }
  if (!motor_svpwm_solve(&config->svpwm, &state->v_ab, &state->modulation))
  {
    return false;
  }
  state->duty[0] = state->modulation.duty[0];
  state->duty[1] = state->modulation.duty[1];
  state->duty[2] = state->modulation.duty[2];
  if (state->modulation.limited)
  {
    state->limited = true;
    ++state->limit_count;
  }

  ++state->step_count;

  return true;
}

/**
 * @brief 执行一拍开环电压（V/F）运算并更新占空比。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param i_abc 三相实测电流 A。
 * @param vdq_ref 开环 d-q 电压指令 V。
 * @param theta_rad 本拍电角度 rad。
 * @return 成功返回 true。
 */
bool motor_control_step_voltage(motor_control_state_t *state,
                               const motor_control_config_t *config,
                               const motor_abc_t *i_abc,
                               const motor_dq_t *vdq_ref,
                               float theta_rad)
{
  motor_alpha_beta_t i_ab;
  motor_dq_t command;
  float v_limit;
  float magnitude;
  float scale;

  if ((state == NULL) || !motor_control_validate_config(config) || (i_abc == NULL) ||
      (vdq_ref == NULL))
  {
    return false;
  }
  if (!motor_is_finite(theta_rad) || !motor_is_finite(vdq_ref->d) || !motor_is_finite(vdq_ref->q))
  {
    return false;
  }
  if (!motor_is_finite(i_abc->a) || !motor_is_finite(i_abc->b) || !motor_is_finite(i_abc->c))
  {
    return false;
  }
  if (state->stopped)
  {
    return false;
  }

  if (!motor_clarke(i_abc, &i_ab))
  {
    return false;
  }
  if (!motor_park(&i_ab, theta_rad, &state->idq))
  {
    return false;
  }
  state->i_ab = i_ab;
  if (!motor_is_finite(state->idq.d) || !motor_is_finite(state->idq.q))
  {
    return false;
  }

  /* V/F 直接使用电压指令，PI 与解耦前馈在本通路不参与运算。 */
  command = *vdq_ref;
  state->vdq_ff.d = 0.0f;
  state->vdq_ff.q = 0.0f;

  v_limit = motor_control_voltage_limit(config);
  state->v_limit_v = v_limit;
  state->limited = false;
  if (!motor_is_finite(v_limit) || (v_limit <= 0.0f))
  {
    return false;
  }

  magnitude = sqrtf((command.d * command.d) + (command.q * command.q));
  if (!motor_is_finite(magnitude))
  {
    return false;
  }
  if (magnitude > v_limit)
  {
    scale = v_limit / magnitude;
    command.d *= scale;
    command.q *= scale;
    state->limited = true;
    ++state->limit_count;
  }
  state->vdq = command;

  if (!motor_inverse_park(&state->vdq, theta_rad, &state->v_ab))
  {
    return false;
  }
  if (!motor_svpwm_solve(&config->svpwm, &state->v_ab, &state->modulation))
  {
    return false;
  }
  state->duty[0] = state->modulation.duty[0];
  state->duty[1] = state->modulation.duty[1];
  state->duty[2] = state->modulation.duty[2];
  if (state->modulation.limited)
  {
    state->limited = true;
    ++state->limit_count;
  }

  ++state->step_count;

  return true;
}
