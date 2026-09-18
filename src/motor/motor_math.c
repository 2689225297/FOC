/**
 * @file motor_math.c
 * @brief A5 产品化算法模块公共数学工具函数实现（无硬件依赖）。
 *
 * 仅使用 C11 标准库 math.h 的单精度函数，保证主机与目标板浮点行为一致。
 */

#include "motor/motor_math.h"

#include <math.h>

/**
 * @brief 将角度归一化到 [0, 2π)。
 * @param angle_rad 输入角度（弧度）。
 * @return 归一化后的角度；输入非有限值时返回 0。
 */
float motor_normalize_angle(float angle_rad)
{
  float wrapped;

  if (!motor_is_finite(angle_rad))
  {
    return 0.0f;
  }

  wrapped = fmodf(angle_rad, MOTOR_TWO_PI_F);
  if (wrapped < 0.0f)
  {
    wrapped += MOTOR_TWO_PI_F;
  }
  if (wrapped >= MOTOR_TWO_PI_F)
  {
    wrapped -= MOTOR_TWO_PI_F;
  }

  return wrapped;
}

/**
 * @brief 将角度归一化到 [-π, π)。
 * @param angle_rad 输入角度（弧度）。
 * @return 归一化后的角度；输入非有限值时返回 0。
 */
float motor_wrap_pi(float angle_rad)
{
  float wrapped = motor_normalize_angle(angle_rad);

  if (wrapped >= MOTOR_PI_F)
  {
    wrapped -= MOTOR_TWO_PI_F;
  }

  return wrapped;
}

/**
 * @brief 将数值限幅到 [-limit, +limit]。
 * @param value 输入数值。
 * @param limit 正限幅幅值，负值按 0 处理。
 * @return 限幅后的数值。
 */
float motor_limit_abs(float value, float limit)
{
  if (limit < 0.0f)
  {
    limit = 0.0f;
  }

  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

/**
 * @brief 将数值限幅到 [min_value, max_value]。
 * @param value 输入数值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 限幅后的数值；区间非法（min > max）时返回 min_value。
 */
float motor_clamp(float value, float min_value, float max_value)
{
  if (min_value > max_value)
  {
    return min_value;
  }

  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

/**
 * @brief 判断数值是否为有限值。
 * @param value 输入数值。
 * @return 有限返回 true，NaN 或无穷返回 false。
 */
bool motor_is_finite(float value)
{
  return isfinite(value) ? true : false;
}

/**
 * @brief 线性插值。
 * @param from 起点值。
 * @param to 终点值。
 * @param t 插值比例，限幅到 [0, 1]。
 * @return 插值结果。
 */
float motor_lerp(float from, float to, float t)
{
  float ratio = motor_clamp(t, 0.0f, 1.0f);

  return from + ((to - from) * ratio);
}
