/**
 * @file motor_transform.h
 * @brief A5 产品化算法模块 Clarke/Park 及其反变换的数值接口声明。
 *
 * 全部变换采用等幅（amplitude invariant）约定，输入输出均为标幺或 SI 单位，
 * 函数不持有状态、不访问硬件，返回 false 表示输入指针为空或数值非有限。
 */
#ifndef MOTOR_TRANSFORM_H
#define MOTOR_TRANSFORM_H

#include "motor/motor_math.h"

/** @brief 三相到 α-β 的等幅 Clarke 变换（要求 a+b+c≈0）。 */
bool motor_clarke(const motor_abc_t *abc, motor_alpha_beta_t *alpha_beta);

/** @brief α-β 到三相的等幅反 Clarke 变换（三线制，输出和为零）。 */
bool motor_inverse_clarke(const motor_alpha_beta_t *alpha_beta, motor_abc_t *abc);

/** @brief α-β 到 d-q 的 Park 变换，theta_rad 为电角度。 */
bool motor_park(const motor_alpha_beta_t *alpha_beta, float theta_rad, motor_dq_t *dq);

/** @brief d-q 到 α-β 的反 Park 变换，theta_rad 为电角度。 */
bool motor_inverse_park(const motor_dq_t *dq, float theta_rad, motor_alpha_beta_t *alpha_beta);

/** @brief 判断三相量之和是否在容差内接近零（用于校验反变换结果）。 */
bool motor_abc_sum_is_zero(const motor_abc_t *abc, float tolerance);

#endif
