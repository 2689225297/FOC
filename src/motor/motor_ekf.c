/**
 * @file motor_ekf.c
 * @brief A5 产品化算法模块 EKF 实现（无硬件依赖，静态分配）。
 *
 * 模型在静止 α-β 坐标系上建立：电流状态保留扩展反电动势项（ωe·λ 与 θe 的
 * 旋转耦合），故电流测量直接携带电角度信息、角度通道可观测。电流动态按 q 轴
 * 电感离散（稳态 d 轴电流为零，不主导电流变化），雅可比对 θe 求偏导时计入
 * 电压矢量的旋转项。
 *
 * 预测：x⁻ = f(x, u)，P⁻ = F·P·Fᵀ + Q
 * 更新：S = H·P⁻·Hᵀ + R，K = P⁻·Hᵀ·S⁻¹，x = x⁻ + K·(y - H·x⁻)，
 *       P = P⁻ - K·H·P⁻（H 固定为取前两行，故可裁剪为二维运算）。
 */

#include "motor/motor_ekf.h"

#include "motor/motor_transform.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 填充 EKF 默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_ekf_default_config(motor_ekf_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->rs_ohm = 0.05f;
  config->ld_h = 0.0002f;
  config->lq_h = 0.0005f;
  config->flux_wb = 0.01f;
  config->q_id = 1.0e-3f;
  config->q_iq = 1.0e-3f;
  config->q_we = 1.0e3f;
  config->q_theta = 1.0e-3f;
  config->r_id = 2.5e-3f;
  config->r_iq = 2.5e-3f;
  config->p_init = 1.0f;

  return true;
}

/**
 * @brief 校验 EKF 配置。
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_ekf_validate_config(const motor_ekf_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->rs_ohm) || (config->rs_ohm < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->ld_h) || (config->ld_h <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->lq_h) || (config->lq_h <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->flux_wb) || (config->flux_wb <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->q_id) || (config->q_id < 0.0f) ||
      !motor_is_finite(config->q_iq) || (config->q_iq < 0.0f) ||
      !motor_is_finite(config->q_we) || (config->q_we < 0.0f) ||
      !motor_is_finite(config->q_theta) || (config->q_theta < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->r_id) || (config->r_id <= 0.0f) ||
      !motor_is_finite(config->r_iq) || (config->r_iq <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->p_init) || (config->p_init <= 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 复位 EKF 状态与协方差。
 * @param state EKF 状态。
 * @param config EKF 配置。
 * @param theta0_rad 初始电角度估计 rad。
 * @param omega0_rad_s 初始电角速度估计 rad/s。
 * @return 成功返回 true。
 */
bool motor_ekf_reset(motor_ekf_state_t *state,
                    const motor_ekf_config_t *config,
                    float theta0_rad,
                    float omega0_rad_s)
{
  int row;
  int col;

  if ((state == NULL) || !motor_ekf_validate_config(config))
  {
    return false;
  }
  if (!motor_is_finite(theta0_rad) || !motor_is_finite(omega0_rad_s))
  {
    return false;
  }

  state->x[0] = 0.0f;
  state->x[1] = 0.0f;
  state->x[2] = omega0_rad_s;
  state->x[3] = motor_normalize_angle(theta0_rad);

  for (row = 0; row < 4; row++)
  {
    for (col = 0; col < 4; col++)
    {
      state->p[row][col] = (row == col) ? config->p_init : 0.0f;
    }
  }
  /* 角度通道初值置信度更低。 */
  state->p[3][3] = config->p_init * 100.0f;

  state->innovation_alpha = 0.0f;
  state->innovation_beta = 0.0f;
  state->theta_est_rad = state->x[3];
  state->omega_est_rad_s = state->x[2];
  state->steps = 0UL;

  return true;
}

/**
 * @brief 推进一个 EKF 控制周期。
 * @param state EKF 状态。
 * @param config EKF 配置。
 * @param vdq 估计 d-q 坐标系下的定子电压 V（内部反变换到 α-β）。
 * @param idq_meas 估计 d-q 坐标系下的定子电流测量 A（内部反变换到 α-β）。
 * @param dt_s 控制周期 s。
 * @param theta_out 估计电角度输出。
 * @param omega_out 估计电角速度输出。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_ekf_step(motor_ekf_state_t *state,
                   const motor_ekf_config_t *config,
                   const motor_dq_t *vdq,
                   const motor_dq_t *idq_meas,
                   float dt_s,
                   float *theta_out,
                   float *omega_out)
{
  float x_pred[4];
  float p_pred[4][4];
  float temp[4][4];
  motor_alpha_beta_t v_ab;
  motor_alpha_beta_t i_ab_meas;
  float sine;
  float cosine;
  float f[4][4];
  float q_diag[4];
  float s00;
  float s01;
  float s10;
  float s11;
  float det;
  float inv00;
  float inv01;
  float inv10;
  float inv11;
  float k[4][2];
  float innovation[2];
  int row;
  int col;
  int inner;

  if ((state == NULL) || !motor_ekf_validate_config(config))
  {
    return false;
  }
  if ((vdq == NULL) || (idq_meas == NULL) || (theta_out == NULL) || (omega_out == NULL))
  {
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(vdq->d) || !motor_is_finite(vdq->q) ||
      !motor_is_finite(idq_meas->d) || !motor_is_finite(idq_meas->q))
  {
    return false;
  }

  /* 1. 坐标系转换：估计 d-q → 静止 α-β，电压与电流测量同用估计角旋转。 */
  (void)motor_inverse_park(vdq, state->x[3], &v_ab);
  (void)motor_inverse_park(idq_meas, state->x[3], &i_ab_meas);

  sine = sinf(state->x[3]);
  cosine = cosf(state->x[3]);

  /* 2. 状态预测（前向欧拉 + α-β 轴 PMSM 电流方程，含扩展反电动势项）。 */
  x_pred[0] = state->x[0] + (dt_s * ((v_ab.alpha - (config->rs_ohm * state->x[0])) +
                                     (state->x[2] * config->flux_wb * sine)) /
                                     config->lq_h);
  x_pred[1] = state->x[1] + (dt_s * ((v_ab.beta - (config->rs_ohm * state->x[1])) -
                                     (state->x[2] * config->flux_wb * cosine)) /
                                     config->lq_h);
  x_pred[2] = state->x[2];
  x_pred[3] = motor_normalize_angle(state->x[3] + (dt_s * state->x[2]));

  /* 3. 雅可比矩阵 F（对 θe 的偏导计入电压矢量旋转项）。 */
  for (row = 0; row < 4; row++)
  {
    for (col = 0; col < 4; col++)
    {
      f[row][col] = (row == col) ? 1.0f : 0.0f;
    }
  }
  f[0][0] = 1.0f - (dt_s * config->rs_ohm / config->lq_h);
  f[0][2] = dt_s * config->flux_wb * sine / config->lq_h;
  f[0][3] = (dt_s * (((-vdq->d * sine) - (vdq->q * cosine)) +
                     (state->x[2] * config->flux_wb * cosine))) /
            config->lq_h;
  f[1][1] = 1.0f - (dt_s * config->rs_ohm / config->lq_h);
  f[1][2] = -dt_s * config->flux_wb * cosine / config->lq_h;
  f[1][3] = (dt_s * (((vdq->d * cosine) - (vdq->q * sine)) +
                     (state->x[2] * config->flux_wb * sine))) /
            config->lq_h;
  f[3][2] = dt_s;

  /* 4. P⁻ = F·P·Fᵀ + Q。 */
  for (row = 0; row < 4; row++)
  {
    for (col = 0; col < 4; col++)
    {
      float sum = 0.0f;

      for (inner = 0; inner < 4; inner++)
      {
        sum += f[row][inner] * state->p[inner][col];
      }
      temp[row][col] = sum;
    }
  }
  for (row = 0; row < 4; row++)
  {
    for (col = 0; col < 4; col++)
    {
      float sum = 0.0f;

      for (inner = 0; inner < 4; inner++)
      {
        sum += temp[row][inner] * f[col][inner];
      }
      p_pred[row][col] = sum;
    }
  }
  q_diag[0] = config->q_id;
  q_diag[1] = config->q_iq;
  q_diag[2] = config->q_we;
  q_diag[3] = config->q_theta;
  for (row = 0; row < 4; row++)
  {
    p_pred[row][row] += q_diag[row];
  }

  /* 5. 量测更新（H 取前两行，S 为 2×2）。 */
  s00 = p_pred[0][0] + config->r_id;
  s01 = p_pred[0][1];
  s10 = p_pred[1][0];
  s11 = p_pred[1][1] + config->r_iq;
  det = (s00 * s11) - (s01 * s10);

  innovation[0] = i_ab_meas.alpha - x_pred[0];
  innovation[1] = i_ab_meas.beta - x_pred[1];
  state->innovation_alpha = innovation[0];
  state->innovation_beta = innovation[1];

  if (fabsf(det) > 1.0e-12f)
  {
    inv00 = s11 / det;
    inv01 = -s01 / det;
    inv10 = -s10 / det;
    inv11 = s00 / det;

    for (row = 0; row < 4; row++)
    {
      k[row][0] = (p_pred[row][0] * inv00) + (p_pred[row][1] * inv10);
      k[row][1] = (p_pred[row][0] * inv01) + (p_pred[row][1] * inv11);
      state->x[row] = x_pred[row] + (k[row][0] * innovation[0]) + (k[row][1] * innovation[1]);
    }

    for (row = 0; row < 4; row++)
    {
      for (col = 0; col < 4; col++)
      {
        state->p[row][col] = p_pred[row][col] -
                             (k[row][0] * p_pred[0][col]) - (k[row][1] * p_pred[1][col]);
      }
    }
  }
  else
  {
    for (row = 0; row < 4; row++)
    {
      state->x[row] = x_pred[row];
      for (col = 0; col < 4; col++)
      {
        state->p[row][col] = p_pred[row][col];
      }
    }
  }

  state->x[3] = motor_normalize_angle(state->x[3]);
  state->theta_est_rad = state->x[3];
  state->omega_est_rad_s = state->x[2];
  state->steps++;

  *theta_out = state->theta_est_rad;
  *omega_out = state->omega_est_rad_s;

  return true;
}
