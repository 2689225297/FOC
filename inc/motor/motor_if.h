/**
 * @file motor_if.h
 * @brief A5 产品化算法模块 I/F 强拖启动接口声明。
 *
 * I/F 启动分两段：先按预定位时间把电流矢量锁定在给定电角度，再按电流斜坡与
 * 频率斜坡强制升速；频率达到切换阈值后置 ready_to_close，供后续切换到闭环
 * 观测器。全部为数值行为，不涉及硬件与功率输出。
 */
#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include "motor/motor_math.h"

/** @brief I/F 启动配置。 */
typedef struct
{
  float align_time_s;      /**< 预定位时间 s。 */
  float align_current_a;   /**< 预定位与启动电流幅值 A。 */
  float current_rate_a_s;  /**< 电流幅值上升率 A/s（预定位后生效）。 */
  float freq_start_hz;     /**< 起始电频率 Hz。 */
  float freq_rate_hz_s;    /**< 电频率上升率 Hz/s。 */
  float freq_end_hz;       /**< 切换闭环的电频率阈值 Hz。 */
} motor_if_config_t;

/** @brief I/F 启动运行状态。 */
typedef struct
{
  float current_ref_a;  /**< 当前强制电流幅值 A。 */
  float freq_hz;        /**< 当前强制电频率 Hz。 */
  float theta_e_rad;    /**< 强制电角度 rad（归一化到 [0, 2π)）。 */
  float omega_e_rad_s;  /**< 强制电角速度 rad/s。 */
  float elapsed_s;      /**< 累计运行时间 s。 */
  bool align_phase;     /**< 是否仍处于预定位阶段。 */
  bool ready_to_close;  /**< 是否达到切换闭环条件。 */
} motor_if_state_t;

/** @brief 填充默认配置：100ms 预定位、0.5A、2Hz 起升、到 40Hz 可切换。 */
bool motor_if_default_config(motor_if_config_t *config);

/** @brief 校验配置合法性。 */
bool motor_if_validate_config(const motor_if_config_t *config);

/** @brief 复位运行状态并按配置初始化电流与频率。 */
bool motor_if_reset(motor_if_state_t *state, const motor_if_config_t *config);

/** @brief 推进一个控制周期，输出的 d-q 电流指令为 (I, 0)。 */
bool motor_if_step(motor_if_state_t *state,
                  const motor_if_config_t *config,
                  float dt_s,
                  motor_dq_t *idq_ref);

#endif
