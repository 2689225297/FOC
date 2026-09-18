/**
 * @file motor_svpwm.c
 * @brief A5 产品化算法模块 SVPWM 调制实现（无硬件依赖，不写寄存器）。
 *
 * 实现要点：
 * 1. 线性区边界为 Udc/√3，超出后按比例缩放并置 limited 标志；
 * 2. 采用零序注入（max+min 中点）等效 SVPWM，避免逐扇区查表；
 * 3. 死区与最小脉宽约束折算为最小占空比份额并钳制，置 dead_time_clamped；
 * 4. 不产生任何功率输出，仅输出占空比数值供主机测试校验。
 */

#include "motor/motor_svpwm.h"

#include <math.h>
#include <stddef.h>

/** @brief 认为达到线性区边界的调制比阈值。 */
#define MOTOR_SVPWM_FULL_MOD_INDEX 0.999f

/**
 * @brief 填充默认配置。
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_svpwm_default_config(motor_svpwm_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  config->carrier_hz = 20000.0f;
  config->bus_voltage_v = 24.0f;
  config->dead_time_s = 500.0e-9f;
  config->min_pulse_s = 1000.0e-9f;

  return true;
}

/**
 * @brief 校验 SVPWM 配置合法性。
 * @param config 待校验配置。
 * @return 载波、母线电压与时间约束均合法返回 true。
 */
bool motor_svpwm_validate_config(const motor_svpwm_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_is_finite(config->carrier_hz) || (config->carrier_hz <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->bus_voltage_v) || (config->bus_voltage_v <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->dead_time_s) || (config->dead_time_s < 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(config->min_pulse_s) || (config->min_pulse_s < 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 计算线性区最大 α-β 电压矢量幅值。
 * @param config SVPWM 配置。
 * @return Udc/√3；配置非法返回 0。
 */
float motor_svpwm_max_linear_voltage(const motor_svpwm_config_t *config)
{
  if (!motor_svpwm_validate_config(config))
  {
    return 0.0f;
  }

  return config->bus_voltage_v * MOTOR_INV_SQRT3_F;
}

/**
 * @brief 计算由死区与最小脉宽决定的最小占空比份额。
 * @param config SVPWM 配置。
 * @return 最小占空比（0..0.5）；配置非法返回 0。
 */
float motor_svpwm_min_duty(const motor_svpwm_config_t *config)
{
  float duty;

  if (!motor_svpwm_validate_config(config))
  {
    return 0.0f;
  }

  duty = (config->dead_time_s + config->min_pulse_s) * config->carrier_hz;
  if (duty < 0.0f)
  {
    duty = 0.0f;
  }
  if (duty > 0.5f)
  {
    duty = 0.5f;
  }

  return duty;
}

/**
 * @brief 求解一周期三相占空比。
 * @param config SVPWM 配置。
 * @param v_ref α-β 参考电压。
 * @param result 求解结果输出。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_svpwm_solve(const motor_svpwm_config_t *config,
                      const motor_alpha_beta_t *v_ref,
                      motor_svpwm_result_t *result)
{
  float v_alpha;
  float v_beta;
  float magnitude;
  float v_limit;
  float scale = 1.0f;
  float phase[3];
  float phase_max;
  float phase_min;
  float offset;
  float duty_min;
  int index;

  if (!motor_svpwm_validate_config(config) || (v_ref == NULL) || (result == NULL))
  {
    return false;
  }
  if (!motor_is_finite(v_ref->alpha) || !motor_is_finite(v_ref->beta))
  {
    return false;
  }

  v_limit = motor_svpwm_max_linear_voltage(config);
  v_alpha = v_ref->alpha;
  v_beta = v_ref->beta;
  magnitude = sqrtf((v_alpha * v_alpha) + (v_beta * v_beta));

  result->limited = false;
  if (magnitude > v_limit)
  {
    scale = v_limit / magnitude;
    v_alpha *= scale;
    v_beta *= scale;
    magnitude = v_limit;
    result->limited = true;
  }

  result->v_max_linear = v_limit;
  result->modulation_index = (v_limit > 0.0f) ? (magnitude / v_limit) : 0.0f;
  result->near_full_modulation = (result->modulation_index >= MOTOR_SVPWM_FULL_MOD_INDEX);

  phase[0] = v_alpha;
  phase[1] = ((-v_alpha) + (MOTOR_SQRT3_F * v_beta)) * 0.5f;
  phase[2] = ((-v_alpha) - (MOTOR_SQRT3_F * v_beta)) * 0.5f;

  phase_max = phase[0];
  phase_min = phase[0];
  for (index = 1; index < 3; index++)
  {
    if (phase[index] > phase_max)
    {
      phase_max = phase[index];
    }
    if (phase[index] < phase_min)
    {
      phase_min = phase[index];
    }
  }

  offset = -0.5f * (phase_max + phase_min);
  duty_min = motor_svpwm_min_duty(config);

  result->dead_time_clamped = false;
  for (index = 0; index < 3; index++)
  {
    float duty = 0.5f + ((phase[index] + offset) / config->bus_voltage_v);

    if (duty < duty_min)
    {
      duty = duty_min;
      result->dead_time_clamped = true;
    }
    else if (duty > (1.0f - duty_min))
    {
      duty = 1.0f - duty_min;
      result->dead_time_clamped = true;
    }
    else
    {
      /* 占空比在允许范围内，无需钳制。 */
    }

    result->duty[index] = duty;
  }

  return true;
}

/**
 * @brief 返回参考电压所在扇区。
 * @param v_ref α-β 参考电压。
 * @return 扇区号 1..6；零矢量或非法输入返回 0。
 */
int motor_svpwm_sector(const motor_alpha_beta_t *v_ref)
{
  float magnitude;
  float angle;
  int sector;

  if (v_ref == NULL)
  {
    return 0;
  }
  if (!motor_is_finite(v_ref->alpha) || !motor_is_finite(v_ref->beta))
  {
    return 0;
  }

  magnitude = sqrtf((v_ref->alpha * v_ref->alpha) + (v_ref->beta * v_ref->beta));
  if (magnitude <= 1.0e-9f)
  {
    return 0;
  }

  angle = atan2f(v_ref->beta, v_ref->alpha);
  if (angle < 0.0f)
  {
    angle += MOTOR_TWO_PI_F;
  }

  sector = (int)(angle / (MOTOR_PI_F / 3.0f)) + 1;
  if (sector > 6)
  {
    sector = 6;
  }
  if (sector < 1)
  {
    sector = 1;
  }

  return sector;
}

/**
 * @brief 由三相占空比反解 α-β 参考电压。
 * @param config SVPWM 配置。
 * @param duty 三相占空比数组。
 * @param v_out 反解出的 α-β 电压输出。
 * @return 成功返回 true，参数非法返回 false。
 */
bool motor_svpwm_duty_to_alpha_beta(const motor_svpwm_config_t *config,
                                   const float duty[3],
                                   motor_alpha_beta_t *v_out)
{
  float phase[3];
  float common;
  int index;

  if (!motor_svpwm_validate_config(config) || (duty == NULL) || (v_out == NULL))
  {
    return false;
  }

  for (index = 0; index < 3; index++)
  {
    if (!motor_is_finite(duty[index]))
    {
      return false;
    }
    phase[index] = (duty[index] - 0.5f) * config->bus_voltage_v;
  }

  common = (phase[0] + phase[1] + phase[2]) / 3.0f;
  v_out->alpha = phase[0] - common;
  v_out->beta = (phase[1] - phase[2]) * MOTOR_INV_SQRT3_F;

  return true;
}
