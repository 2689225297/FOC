/**
 * @file motor_smo.c
 * @brief A5 产品化算法模块滑模观测器实现（无硬件依赖）。
 *
 * 离散模型：
 *   s      = i_est - i_meas
 *   z      = k · sat(s / φ)
 *   i_est += dt/L · (v - Rs·i_est - z)
 *   e_est += dt·ωc · (z - e_est)        （z 的低通即等效控制，幅值为反电动势）
 *   θ_raw  = atan2(-e_α, e_β)            （相对电角度存在 π 模糊）
 *   PLL:  ω_est += ki·wrap(θ_raw-θ_est)·dt, θ_est += (ω_est + kp·wrap)·dt
 *
 * 工程约束：
 *   1. θ_raw 只能确定角度模 π，反转时需按反电动势矢量角速度符号补 π；
 *   2. PLL 捕获范围有限，冷启动大相位误差会造成周期滑移，故在反电动势连续
 *      有效 MOTOR_SMO_ALIGN_TIME_S 后一次性对齐相位与频率再投入 PLL；
 *   3. 电流估计在首个周期用实测电流初始化，避免电流阶跃经 k 倍注入放大后
 *      污染反电动势低通，使低速段误判为有效观测；
 *   4. 反电动势幅值低于门限时置 converged=false，角度保持、速度保持在积分器。
 */

#include "motor/motor_smo.h"

#include <math.h>
#include <stddef.h>

/** @brief 反电动势角速度低通时间常数 s。 */
#define MOTOR_SMO_RATE_TAU_S 0.0005f

/** @brief 相位对齐前要求反电动势连续有效的时长 s。 */
#define MOTOR_SMO_ALIGN_TIME_S 0.005f

/** @brief 饱和函数实现，超出边界层线性限幅。 */
static float motor_smo_saturate(float value, float boundary)
{
  float ratio;

  if (boundary <= 0.0f)
  {
    return (value >= 0.0f) ? 1.0f : -1.0f;
  }

  ratio = value / boundary;
  if (ratio > 1.0f)
  {
    ratio = 1.0f;
  }
  else if (ratio < -1.0f)
  {
    ratio = -1.0f;
  }
  else
  {
    /* 边界层内保持线性，抑制抖振。 */
  }

  return ratio;
}

/**
 * @brief 填充 SMO 默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_smo_default_config(motor_smo_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->rs_ohm = 0.05f;
  config->l_h = 0.00035f;
  config->slide_gain_v = 5.0f;
  /* 边界层下界由离散注入稳定性给出：α·(Rs + k/φ) < 2 → φ > 0.36A（dt=50µs）。 */
  config->boundary_a = 1.0f;
  config->emf_lpf_hz = 500.0f;
  config->pll_kp = 300.0f;
  config->pll_ki = 20000.0f;
  config->emf_min_v = 0.1f;

  return true;
}

/**
 * @brief 校验 SMO 配置。
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_smo_validate_config(const motor_smo_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->rs_ohm) || (config->rs_ohm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->l_h) || (config->l_h <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->slide_gain_v) || (config->slide_gain_v <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->boundary_a) || (config->boundary_a <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->emf_lpf_hz) || (config->emf_lpf_hz <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->pll_kp) || !motor_is_finite(config->pll_ki))
  {
    return false;
  }
  if (!motor_is_finite(config->emf_min_v) || (config->emf_min_v < 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位 SMO 状态。
 * @param state SMO 状态。
 * @return 状态指针非空返回 true。
 */
bool motor_smo_reset(motor_smo_state_t *state)
{
  if (state == NULL)
  {
    return false;
  }

  state->i_est.alpha = 0.0f;
  state->i_est.beta = 0.0f;
  state->emf_est.alpha = 0.0f;
  state->emf_est.beta = 0.0f;
  state->theta_est_rad = 0.0f;
  state->omega_est_rad_s = 0.0f;
  state->pll_integral = 0.0f;
  state->slide_surface = 0.0f;
  state->emf_mag_v = 0.0f;
  state->converged = false;
  state->theta_raw_prev = 0.0f;
  state->emf_rate_rad_s = 0.0f;
  state->emf_valid_s = 0.0f;
  state->pll_locked = false;
  state->i_est_valid = false;

  return true;
}

/**
 * @brief 由 α-β 反电动势解析电角度。
 * @param emf_ab α-β 反电动势 V。
 * @return 电角度 rad；输入非法返回 0。
 */
float motor_smo_angle_from_emf(const motor_alpha_beta_t *emf_ab)
{
  if (emf_ab == NULL)
  {
    return 0.0f;
  }
  if (!motor_is_finite(emf_ab->alpha) || !motor_is_finite(emf_ab->beta))
  {
    return 0.0f;
  }

  return motor_normalize_angle(atan2f(-emf_ab->alpha, emf_ab->beta));
}

/**
 * @brief 推进一个控制周期。
 * @param state SMO 状态。
 * @param config SMO 配置。
 * @param v_ab α-β 电压 V。
 * @param i_ab α-β 电流 A。
 * @param dt_s 控制周期 s。
 * @param theta_out 估计电角度输出。
 * @param omega_out 估计电角速度输出。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_smo_step(motor_smo_state_t *state,
                   const motor_smo_config_t *config,
                   const motor_alpha_beta_t *v_ab,
                   const motor_alpha_beta_t *i_ab,
                   float dt_s,
                   float *theta_out,
                   float *omega_out)
{
  float s_alpha;
  float s_beta;
  float z_alpha;
  float z_beta;
  float wc;
  float theta_raw;
  float theta_error;
  float rate_alpha;

  if ((state == NULL) || !motor_smo_validate_config(config))
  {
    return false;
  }
  if ((v_ab == NULL) || (i_ab == NULL) || (theta_out == NULL) || (omega_out == NULL))
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

  if (!state->i_est_valid)
  {
    /* 冷启动对齐：首个周期用实测电流初始化电流估计。 */
    state->i_est = *i_ab;
    state->i_est_valid = true;
  }

  s_alpha = state->i_est.alpha - i_ab->alpha;
  s_beta = state->i_est.beta - i_ab->beta;
  state->slide_surface = sqrtf((s_alpha * s_alpha) + (s_beta * s_beta));

  z_alpha = config->slide_gain_v * motor_smo_saturate(s_alpha, config->boundary_a);
  z_beta = config->slide_gain_v * motor_smo_saturate(s_beta, config->boundary_a);

  state->i_est.alpha += (dt_s / config->l_h) *
                        ((v_ab->alpha - (config->rs_ohm * state->i_est.alpha)) - z_alpha);
  state->i_est.beta += (dt_s / config->l_h) *
                       ((v_ab->beta - (config->rs_ohm * state->i_est.beta)) - z_beta);

  wc = MOTOR_TWO_PI_F * config->emf_lpf_hz;
  state->emf_est.alpha += dt_s * wc * (z_alpha - state->emf_est.alpha);
  state->emf_est.beta += dt_s * wc * (z_beta - state->emf_est.beta);

  state->emf_mag_v = sqrtf((state->emf_est.alpha * state->emf_est.alpha) +
                           (state->emf_est.beta * state->emf_est.beta));
  state->converged = (state->emf_mag_v >= config->emf_min_v);

  if (state->converged)
  {
    theta_raw = motor_smo_angle_from_emf(&state->emf_est);

    /* 反电动势矢量角速度低通估计：符号即电角度旋转方向。 */
    rate_alpha = dt_s / (dt_s + MOTOR_SMO_RATE_TAU_S);
    state->emf_rate_rad_s += rate_alpha * ((motor_wrap_pi(theta_raw - state->theta_raw_prev) / dt_s) -
                                           state->emf_rate_rad_s);
    state->theta_raw_prev = theta_raw;
    state->emf_valid_s += dt_s;

    if (!state->pll_locked && (state->emf_valid_s >= MOTOR_SMO_ALIGN_TIME_S))
    {
      /* 一次性对齐：相位由 θ_raw 给出，频率由反电动势角速度前馈。 */
      state->theta_est_rad = (state->emf_rate_rad_s < 0.0f)
                                 ? motor_normalize_angle(theta_raw + MOTOR_PI_F)
                                 : theta_raw;
      state->pll_integral = state->emf_rate_rad_s;
      state->omega_est_rad_s = state->emf_rate_rad_s;
      state->pll_locked = true;
    }

    if (state->pll_locked)
    {
      if (state->emf_rate_rad_s < 0.0f)
      {
        /* 反转时 atan2 结果与真实电角度相差 π，由旋转方向消歧。 */
        theta_raw = motor_normalize_angle(theta_raw + MOTOR_PI_F);
      }
      theta_error = motor_wrap_pi(theta_raw - state->theta_est_rad);

      state->pll_integral += config->pll_ki * theta_error * dt_s;
      state->omega_est_rad_s = state->pll_integral + (config->pll_kp * theta_error);
      state->theta_est_rad = motor_normalize_angle(
          state->theta_est_rad + (state->omega_est_rad_s * dt_s));
    }
  }
  else
  {
    /* 反电动势不足：复位对齐流程，保持上一次角度，速度保持在积分器上。 */
    state->emf_valid_s = 0.0f;
    state->emf_rate_rad_s = 0.0f;
    state->pll_locked = false;
    state->omega_est_rad_s = state->pll_integral;
  }

  *theta_out = state->theta_est_rad;
  *omega_out = state->omega_est_rad_s;

  return true;
}
