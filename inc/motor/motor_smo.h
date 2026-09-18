/**
 * @file motor_smo.h
 * @brief A5 产品化算法模块滑模观测器（SMO）接口声明。
 *
 * 采用 α-β 轴电流滑模观测 + 反电动势低通提取 + PLL 锁相的结构。滑模面为
 * 估计电流与实测电流之差，等效控制量经低通得到反电动势，再经 PLL 得到电角度
 * 与电角速度。低速段反电动势幅值低于门限时置 converged=false，用于量化 SMO
 * 的可用转速下界与参数误差敏感性。
 *
 * 选型约束：SMO 作为低计算量回退与交叉校验方案，须标定 Rs 与有效门限；在
 * 反电动势门限邻近区（约 10~20rad/s）不得直接用于观测器切换，门限以下不可用。
 */
#ifndef MOTOR_SMO_H
#define MOTOR_SMO_H

#include "motor/motor_math.h"

/** @brief SMO 配置。 */
typedef struct
{
  float rs_ohm;        /**< 定子电阻估计值 Ω。 */
  float l_h;           /**< 同步电感估计值 H（取 (Ld+Lq)/2）。 */
  float slide_gain_v;  /**< 滑模等效控制增益 V。 */
  float boundary_a;    /**< 饱和函数边界层 A。 */
  float emf_lpf_hz;    /**< 反电动势低通截止频率 Hz。 */
  float pll_kp;        /**< PLL 比例增益。 */
  float pll_ki;        /**< PLL 积分增益。 */
  float emf_min_v;     /**< 判定观测有效的反电动势幅值门限 V。 */
} motor_smo_config_t;

/** @brief SMO 状态。 */
typedef struct
{
  motor_alpha_beta_t i_est;   /**< 估计 α-β 电流 A。 */
  motor_alpha_beta_t emf_est; /**< 估计 α-β 反电动势 V。 */
  float theta_est_rad;       /**< 估计电角度 rad。 */
  float omega_est_rad_s;     /**< 估计电角速度 rad/s。 */
  float pll_integral;        /**< PLL 积分项。 */
  float slide_surface;       /**< 滑模面幅值 A。 */
  float emf_mag_v;           /**< 反电动势幅值 V。 */
  float theta_raw_prev;      /**< 上一拍反电动势解析电角度 rad。 */
  float emf_rate_rad_s;      /**< 反电动势矢量角速度低通估计 rad/s，符号即旋转方向。 */
  float emf_valid_s;         /**< 反电动势连续有效时长 s。 */
  bool converged;            /**< 反电动势是否超过有效门限。 */
  bool pll_locked;           /**< PLL 是否已完成相位对齐并投入运行。 */
  bool i_est_valid;          /**< 电流估计是否已由实测电流初始化。 */
} motor_smo_state_t;

/** @brief 填充默认配置（0.05Ω、0.35mH、增益 5V、边界层 1.0A、500Hz、PLL 300/20000）。 */
bool motor_smo_default_config(motor_smo_config_t *config);

/** @brief 校验 SMO 配置合法性。 */
bool motor_smo_validate_config(const motor_smo_config_t *config);

/** @brief 复位 SMO 状态。 */
bool motor_smo_reset(motor_smo_state_t *state);

/** @brief 推进一个控制周期，输出估计电角度与电角速度。 */
bool motor_smo_step(motor_smo_state_t *state,
                   const motor_smo_config_t *config,
                   const motor_alpha_beta_t *v_ab,
                   const motor_alpha_beta_t *i_ab,
                   float dt_s,
                   float *theta_out,
                   float *omega_out);

/** @brief 由 α-β 反电动势解析计算电角度（PMSM 约定）。 */
float motor_smo_angle_from_emf(const motor_alpha_beta_t *emf_ab);

#endif
