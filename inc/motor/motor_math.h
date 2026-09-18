/**
 * @file motor_math.h
 * @brief A5 产品化算法模块公共数学常量、向量类型与基础工具函数声明。
 *
 * 本文件属于产品级算法模块，只依赖 C11 标准库，不依赖厂商 DFP 头文件，
 * 并可与 inc/motor/motor_types.h 的冻结类型共存。全部函数为纯数值计算，不访问寄存器、
 * 不输出功率、不依赖任何硬件，可直接在主机单元测试中运行。
 */
#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H

#include <stdbool.h>

/** @brief 圆周率（单精度近似）。 */
#define MOTOR_PI_F 3.14159265358979323846f
/** @brief 2π（单精度近似）。 */
#define MOTOR_TWO_PI_F 6.28318530717958647692f
/** @brief π/2（单精度近似）。 */
#define MOTOR_HALF_PI_F 1.57079632679489661923f
/** @brief √3（单精度近似）。 */
#define MOTOR_SQRT3_F 1.73205080756887729353f
/** @brief 1/√3（单精度近似）。 */
#define MOTOR_INV_SQRT3_F 0.57735026918962576451f

/** @brief α-β 静止坐标系分量。 */
typedef struct
{
  float alpha;
  float beta;
} motor_alpha_beta_t;

/** @brief d-q 旋转坐标系分量。 */
typedef struct
{
  float d;
  float q;
} motor_dq_t;

/** @brief a-b-c 三相坐标系分量。 */
typedef struct
{
  float a;
  float b;
  float c;
} motor_abc_t;

/** @brief 将角度归一化到 [0, 2π)，非有限值返回 0。 */
float motor_normalize_angle(float angle_rad);

/** @brief 将角度归一化到 [-π, π)，非有限值返回 0。 */
float motor_wrap_pi(float angle_rad);

/** @brief 将数值限幅到 [-limit, +limit]，limit 为负时取 0。 */
float motor_limit_abs(float value, float limit);

/** @brief 将数值限幅到 [min_value, max_value]，区间非法时返回 min_value。 */
float motor_clamp(float value, float min_value, float max_value);

/** @brief 判断数值是否为有限值（非 NaN 且非无穷）。 */
bool motor_is_finite(float value);

/** @brief 线性插值，t 被限幅到 [0, 1]。 */
float motor_lerp(float from, float to, float t);

#endif
