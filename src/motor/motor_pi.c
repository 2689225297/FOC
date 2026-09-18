/**
 * @file motor_pi.c
 * @brief A5 产品化算法模块 d-q 轴 PI 电流调节器与解耦前馈实现。
 *
 * 无硬件依赖，用于产品级回归：阶跃响应、限幅行为、抗积分饱和与解耦项符号
 * 正确性。
 */

#include "motor/motor_pi.h"

#include <stddef.h>

/**
 * @brief 初始化 PI 调节器。
 * @param pi 调节器实例。
 * @param kp 比例增益。
 * @param ki 积分增益。
 * @param out_min 输出下限。
 * @param out_max 输出上限。
 */
void motor_pi_init(motor_pi_t *pi, float kp, float ki, float out_min, float out_max)
{
  if (pi == NULL)
  {
    return;
  }

  pi->kp = kp;
  pi->ki = ki;
  pi->out_min = out_min;
  pi->out_max = out_max;
  pi->integral_limit = (out_max > -out_min) ? out_max : -out_min;
  if (pi->integral_limit < 0.0f)
  {
    pi->integral_limit = -pi->integral_limit;
  }
  motor_pi_reset(pi);
}

/**
 * @brief 重置 PI 调节器状态。
 * @param pi 调节器实例。
 */
void motor_pi_reset(motor_pi_t *pi)
{
  if (pi == NULL)
  {
    return;
  }

  pi->integral = 0.0f;
  pi->saturated = false;
  pi->sat_count = 0U;
}

/**
 * @brief 设置积分项绝对限幅。
 * @param pi 调节器实例。
 * @param integral_limit 限幅值，负值取绝对值。
 */
void motor_pi_set_integral_limit(motor_pi_t *pi, float integral_limit)
{
  if (pi == NULL)
  {
    return;
  }

  pi->integral_limit = (integral_limit < 0.0f) ? -integral_limit : integral_limit;
}

/**
 * @brief 执行一拍 PI 运算。
 * @param pi 调节器实例。
 * @param error 当前误差。
 * @param dt 控制周期 s。
 * @return 限幅后的输出。
 */
float motor_pi_step(motor_pi_t *pi, float error, float dt)
{
  float candidate;
  float raw;
  float limited;

  if (pi == NULL)
  {
    return 0.0f;
  }
  if (!motor_is_finite(error) || !motor_is_finite(dt) || (dt <= 0.0f))
  {
    return motor_clamp(pi->integral, pi->out_min, pi->out_max);
  }

  candidate = pi->integral + (pi->ki * error * dt);
  candidate = motor_limit_abs(candidate, pi->integral_limit);
  raw = (pi->kp * error) + candidate;
  limited = motor_clamp(raw, pi->out_min, pi->out_max);

  if ((raw > pi->out_max) && (error > 0.0f))
  {
    /* 输出饱和且误差继续加深：冻结积分，避免积分饱和。 */
    pi->saturated = true;
    pi->sat_count++;
  }
  else if ((raw < pi->out_min) && (error < 0.0f))
  {
    /* 反向饱和且误差继续加深：同样冻结积分。 */
    pi->saturated = true;
    pi->sat_count++;
  }
  else
  {
    pi->integral = candidate;
    pi->saturated = false;
  }

  return limited;
}

/**
 * @brief 计算 d-q 轴解耦前馈电压。
 * @param params 电机参数。
 * @param idq 当前 d-q 电流。
 * @param omega_e_rad_s 电角速度。
 * @param v_ff 前馈电压输出。
 */
void motor_decoupling_feedforward(const motor_motor_params_t *params,
                                 const motor_dq_t *idq,
                                 float omega_e_rad_s,
                                 motor_dq_t *v_ff)
{
  if ((params == NULL) || (idq == NULL) || (v_ff == NULL))
  {
    return;
  }
  if (!motor_is_finite(omega_e_rad_s) || !motor_is_finite(idq->d) || !motor_is_finite(idq->q))
  {
    v_ff->d = 0.0f;
    v_ff->q = 0.0f;
    return;
  }

  v_ff->d = -omega_e_rad_s * params->lq_h * idq->q;
  v_ff->q = omega_e_rad_s * ((params->ld_h * idq->d) + params->flux_wb);
}
