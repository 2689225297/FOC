/**
 * @file motor_transform.c
 * @brief A5 产品化算法模块 Clarke/Park 及其反变换实现（无硬件依赖）。
 *
 * 变换矩阵使用电机域公共类型，与产品冻结接口解耦，参与 A5 产品级回归。
 */

#include "motor/motor_transform.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 三相到 α-β 的等幅 Clarke 变换。
 * @param abc 三相输入，要求 a+b+c 接近零。
 * @param alpha_beta 变换结果输出。
 * @return 成功返回 true，指针为空或数值非有限返回 false。
 */
bool motor_clarke(const motor_abc_t *abc, motor_alpha_beta_t *alpha_beta)
{
  if ((abc == NULL) || (alpha_beta == NULL))
  {
    return false;
  }
  if (!motor_is_finite(abc->a) || !motor_is_finite(abc->b) || !motor_is_finite(abc->c))
  {
    return false;
  }

  alpha_beta->alpha = abc->a;
  alpha_beta->beta = (abc->a + (2.0f * abc->b)) * MOTOR_INV_SQRT3_F;

  return true;
}

/**
 * @brief α-β 到三相的等幅反 Clarke 变换。
 * @param alpha_beta α-β 输入。
 * @param abc 三相输出，满足 a+b+c=0。
 * @return 成功返回 true，指针为空或数值非有限返回 false。
 */
bool motor_inverse_clarke(const motor_alpha_beta_t *alpha_beta, motor_abc_t *abc)
{
  if ((alpha_beta == NULL) || (abc == NULL))
  {
    return false;
  }
  if (!motor_is_finite(alpha_beta->alpha) || !motor_is_finite(alpha_beta->beta))
  {
    return false;
  }

  abc->a = alpha_beta->alpha;
  abc->b = ((-alpha_beta->alpha) + (MOTOR_SQRT3_F * alpha_beta->beta)) * 0.5f;
  abc->c = ((-alpha_beta->alpha) - (MOTOR_SQRT3_F * alpha_beta->beta)) * 0.5f;

  return true;
}

/**
 * @brief α-β 到 d-q 的 Park 变换。
 * @param alpha_beta α-β 输入。
 * @param theta_rad 电角度（弧度）。
 * @param dq d-q 输出。
 * @return 成功返回 true，指针为空或数值非有限返回 false。
 */
bool motor_park(const motor_alpha_beta_t *alpha_beta, float theta_rad, motor_dq_t *dq)
{
  float cos_theta;
  float sin_theta;

  if ((alpha_beta == NULL) || (dq == NULL))
  {
    return false;
  }
  if (!motor_is_finite(alpha_beta->alpha) || !motor_is_finite(alpha_beta->beta) ||
      !motor_is_finite(theta_rad))
  {
    return false;
  }

  cos_theta = cosf(theta_rad);
  sin_theta = sinf(theta_rad);

  dq->d = (alpha_beta->alpha * cos_theta) + (alpha_beta->beta * sin_theta);
  dq->q = (-alpha_beta->alpha * sin_theta) + (alpha_beta->beta * cos_theta);

  return true;
}

/**
 * @brief d-q 到 α-β 的反 Park 变换。
 * @param dq d-q 输入。
 * @param theta_rad 电角度（弧度）。
 * @param alpha_beta α-β 输出。
 * @return 成功返回 true，指针为空或数值非有限返回 false。
 */
bool motor_inverse_park(const motor_dq_t *dq, float theta_rad, motor_alpha_beta_t *alpha_beta)
{
  float cos_theta;
  float sin_theta;

  if ((dq == NULL) || (alpha_beta == NULL))
  {
    return false;
  }
  if (!motor_is_finite(dq->d) || !motor_is_finite(dq->q) || !motor_is_finite(theta_rad))
  {
    return false;
  }

  cos_theta = cosf(theta_rad);
  sin_theta = sinf(theta_rad);

  alpha_beta->alpha = (dq->d * cos_theta) - (dq->q * sin_theta);
  alpha_beta->beta = (dq->d * sin_theta) + (dq->q * cos_theta);

  return true;
}

/**
 * @brief 判断三相量之和是否在容差内接近零。
 * @param abc 三相输入。
 * @param tolerance 允许的绝对容差，负值按 0 处理。
 * @return 在容差内返回 true，指针为空返回 false。
 */
bool motor_abc_sum_is_zero(const motor_abc_t *abc, float tolerance)
{
  float sum;

  if (abc == NULL)
  {
    return false;
  }

  sum = abc->a + abc->b + abc->c;

  return fabsf(sum) <= fabsf(tolerance) ? true : false;
}
