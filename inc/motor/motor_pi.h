/**
 * @file motor_pi.h
 * @brief A5 产品化算法模块 d-q 轴 PI 电流调节器与解耦前馈接口声明。
 *
 * PI 采用条件积分（conditional integration）抗积分饱和：输出饱和且误差继续
 * 加深饱和时冻结积分，误差反向时立即恢复积分更新。解耦前馈按 PMSM 电压方程
 * 计算 vd/vq 交叉耦合项，供产品级电流环使用。
 */
#ifndef MOTOR_PI_H
#define MOTOR_PI_H

#include "motor/motor_math.h"

#include <stdint.h>

/** @brief PI 调节器实例（静态分配，不含动态内存）。 */
typedef struct
{
  float kp;            /**< 比例增益。 */
  float ki;            /**< 积分增益。 */
  float integral;      /**< 积分项累积值。 */
  float out_min;       /**< 输出下限。 */
  float out_max;       /**< 输出上限。 */
  float integral_limit;/**< 积分项绝对限幅。 */
  bool saturated;      /**< 上一拍输出是否被限幅。 */
  uint32_t sat_count;  /**< 饱和拍累计计数。 */
} motor_pi_t;

/** @brief PMSM 电气参数（必须来自已归档的版本化标定记录）。 */
typedef struct
{
  float rs_ohm;  /**< 定子电阻 Ω。 */
  float ld_h;    /**< d 轴电感 H。 */
  float lq_h;    /**< q 轴电感 H。 */
  float flux_wb; /**< 永磁磁链 Wb。 */
} motor_motor_params_t;

/** @brief 初始化 PI 参数并清零积分与统计。 */
void motor_pi_init(motor_pi_t *pi, float kp, float ki, float out_min, float out_max);

/** @brief 重置积分项、饱和标志与饱和计数。 */
void motor_pi_reset(motor_pi_t *pi);

/** @brief 设置积分项限幅（默认取输出限幅幅值）。 */
void motor_pi_set_integral_limit(motor_pi_t *pi, float integral_limit);

/** @brief 执行一拍 PI 运算并返回限幅后的输出。 */
float motor_pi_step(motor_pi_t *pi, float error, float dt);

/** @brief 计算 d-q 轴解耦前馈电压。 */
void motor_decoupling_feedforward(const motor_motor_params_t *params,
                                 const motor_dq_t *idq,
                                 float omega_e_rad_s,
                                 motor_dq_t *v_ff);

#endif
