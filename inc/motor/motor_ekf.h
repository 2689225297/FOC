/**
 * @file motor_ekf.h
 * @brief A5 产品化算法模块扩展卡尔曼滤波器（EKF）接口声明。
 *
 * 状态量 x = [iα, iβ, ωe, θe]，观测量 y = [iα, iβ]。电气方程在静止 α-β
 * 坐标系上建立并保留扩展反电动势项（ωe·λ·sinθe 与 ωe·λ·cosθe），因此电流
 * 测量中直接包含电角度信息、角度通道可观测；雅可比矩阵按一阶离散化推导，
 * 对 θe 的偏导计入电压矢量旋转项。用于验证参数误差、测量噪声与采样延迟对
 * 转速/角度估计的影响，R/Q 未标定前只能用于监测，不得输出闭环角度。
 *
 * 选型约束：EKF 为中高速主观测候选，投放前必须按实测电流噪声重新标定 R/Q
 * （建议 R ≥ 0.1）；R/Q 未标定、或轴参数未版本化时，观测器仲裁禁止选它。
 */
#ifndef MOTOR_EKF_H
#define MOTOR_EKF_H

#include "motor/motor_math.h"

/** @brief EKF 配置。 */
typedef struct
{
  float rs_ohm;    /**< 定子电阻 Ω。 */
  float ld_h;      /**< d 轴电感 H（凸极补偿配置与合法性校验用）。 */
  float lq_h;      /**< q 轴电感 H。 */
  float flux_wb;   /**< 永磁磁链 Wb。 */
  float q_id;      /**< d 轴电流过程噪声方差。 */
  float q_iq;      /**< q 轴电流过程噪声方差。 */
  float q_we;      /**< 电角速度过程噪声方差。 */
  float q_theta;   /**< 电角度过程噪声方差。 */
  float r_id;      /**< d 轴电流测量噪声方差。 */
  float r_iq;      /**< q 轴电流测量噪声方差。 */
  float p_init;    /**< 初始协方差对角值（角度通道自动放大）。 */
} motor_ekf_config_t;

/** @brief EKF 状态。 */
typedef struct
{
  float x[4];              /**< [iα, iβ, ωe, θe]：α-β 电流 A、电角速度 rad/s、电角度 rad。 */
  float p[4][4];           /**< 状态协方差矩阵。 */
  float innovation_alpha;  /**< 最近一次 α 轴电流新息 A。 */
  float innovation_beta;   /**< 最近一次 β 轴电流新息 A。 */
  float theta_est_rad;     /**< 估计电角度 rad。 */
  float omega_est_rad_s;   /**< 估计电角速度 rad/s。 */
  unsigned long steps;     /**< 已推进步数。 */
} motor_ekf_state_t;

/** @brief 填充默认配置（Rs=0.05Ω、Ld=0.2mH、Lq=0.5mH、λ=0.01Wb）。 */
bool motor_ekf_default_config(motor_ekf_config_t *config);

/** @brief 校验 EKF 配置合法性。 */
bool motor_ekf_validate_config(const motor_ekf_config_t *config);

/** @brief 复位 EKF，设定初始电流、转速与角度及初始协方差。 */
bool motor_ekf_reset(motor_ekf_state_t *state,
                    const motor_ekf_config_t *config,
                    float theta0_rad,
                    float omega0_rad_s);

/** @brief 推进一个控制周期：预测 + 量测更新。 */
bool motor_ekf_step(motor_ekf_state_t *state,
                   const motor_ekf_config_t *config,
                   const motor_dq_t *vdq,
                   const motor_dq_t *idq_meas,
                   float dt_s,
                   float *theta_out,
                   float *omega_out);

#endif
