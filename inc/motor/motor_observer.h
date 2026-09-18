/**
 * @file motor_observer.h
 * @brief A5 单轴观测器仲裁与交叉校验接口声明（EKF 主观测、SMO 回退、磁链监测）。
 *
 * 主要接口：配置默认值与校验、状态初始化与复位、单周期推进、角度有效性查询、
 * 闭环放行查询与磁链监测查询。
 * 依赖关系：依赖 motor_ekf、motor_smo、motor_flux 与冻结类型；不访问寄存器、
 * 不分配内存、不做浮点除法以外的硬件相关运算。
 *
 * 选型约束（来自 doc/path-c-observer-conclusions.md）：
 *   1. 中高速主观测优先 EKF，但 R/Q 未按实测电流噪声标定时禁止启用（ekf_enabled
 *      必须为 false，且校验会拒绝 r_id/r_iq 低于 0.1 的配置）；
 *   2. SMO 作为低计算量回退与交叉校验方案，须标定 Rs 与有效门限，反电动势门限
 *      邻近区不得直接用于观测器切换；
 *   3. 磁链观测器只用于中高速磁链幅值监测与参数辨识辅助，不独立承担角度输出，
 *      电频率低于约 10 倍截止频率时监测结论无效；
 *   4. EKF 与 SMO 角度差持续超过门限视为观测器发散，必须锁存并要求安全停机，
 *      禁止用其中一个结果掩盖发散。
 */

#ifndef MOTOR_OBSERVER_H
#define MOTOR_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_ekf.h"
#include "motor/motor_flux.h"
#include "motor/motor_math.h"
#include "motor/motor_smo.h"

/** @brief 当前生效的观测器。 */
typedef enum
{
  MOTOR_OBSERVER_ACTIVE_NONE = 0, /**< 无可信角度源，禁止闭环。 */
  MOTOR_OBSERVER_ACTIVE_EKF,      /**< EKF 主观测。 */
  MOTOR_OBSERVER_ACTIVE_SMO,      /**< SMO 回退与交叉校验。 */
  MOTOR_OBSERVER_ACTIVE_COUNT     /**< 取值数量哨兵。 */
} motor_observer_active_t;

/** @brief 观测器仲裁配置。 */
typedef struct
{
  motor_ekf_config_t ekf;          /**< EKF 配置（含 R/Q 标定值）。 */
  motor_smo_config_t smo;          /**< SMO 配置（含 Rs 与门限标定值）。 */
  motor_flux_config_t flux;        /**< 磁链监测配置。 */
  float pole_pairs;                /**< 极对数，范围 1 到 32。 */
  float flux_wb;                   /**< 已标定永磁磁链 Wb，用于磁链监测期望值。 */
  float min_close_speed_rpm;       /**< 允许闭环输出的最低机械转速 rpm。 */
  float min_ekf_speed_rpm;         /**< EKF 允许作为主观测的最低机械转速 rpm。 */
  float min_smo_speed_rpm;         /**< SMO 允许作为角度源的最低机械转速 rpm。 */
  float cross_check_angle_rad;     /**< EKF 与 SMO 角度差发散门限 rad。 */
  float cross_check_hold_s;        /**< 角度差超门限的持续判定时间 s。 */
  float flux_error_ratio;          /**< 磁链监测相对偏差门限。 */
  bool ekf_enabled;                /**< 仅 R/Q 已按实测噪声标定后置位。 */
  bool smo_enabled;                /**< SMO 是否参与回退与交叉校验。 */
  bool flux_monitor_enabled;       /**< 是否启用磁链幅值监测。 */
  bool parameters_valid;           /**< 观测器参数是否来自已归档的版本化记录。 */
} motor_observer_config_t;

/** @brief 观测器仲裁运行状态。 */
typedef struct
{
  motor_ekf_state_t ekf;              /**< EKF 状态。 */
  motor_smo_state_t smo;              /**< SMO 状态。 */
  motor_flux_state_t flux;            /**< 磁链监测状态。 */
  motor_observer_active_t active;     /**< 当前生效观测器。 */
  float theta_rad;                    /**< 仲裁输出的电角度 rad。 */
  float omega_e_rad_s;                /**< 仲裁输出的电角速度 rad/s。 */
  float speed_rpm;                    /**< 机械转速估计 rpm。 */
  float smo_speed_rpm;                /**< SMO 机械转速估计 rpm。 */
  float angle_error_rad;              /**< EKF 与 SMO 角度差幅值 rad。 */
  float divergence_s;                 /**< 角度差超门限累计时间 s。 */
  float flux_mag_wb;                  /**< 磁链幅值估计 Wb。 */
  float flux_expected_wb;             /**< 按解析衰减计算的期望磁链 Wb。 */
  float flux_error_ratio;             /**< 磁链相对偏差。 */
  uint32_t reject_count;              /**< 仲裁拒绝次数（含发散锁存）。 */
  uint32_t step_count;                /**< 已推进周期数。 */
  bool initialized;                   /**< 是否已复位。 */
  bool angle_valid;                   /**< 当前角度是否可用于闭环。 */
  bool diverged;                      /**< 是否已锁存发散。 */
  bool flux_monitor_ok;               /**< 磁链监测结论是否可用。 */
} motor_observer_state_t;

/**
 * @brief 填充观测器默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 *
 * 调用上下文：初始化前构造候选配置。
 * 失败行为：指针为空返回 false。
 */
bool motor_observer_default_config(motor_observer_config_t *config);

/**
 * @brief 校验观测器配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 *
 * 调用上下文：初始化与参数替换。
 * 失败行为：参数未版本化、EKF 在 R/Q 未标定时被启用、无任何可用角度源或门限
 * 非法时返回 false。
 */
bool motor_observer_validate_config(const motor_observer_config_t *config);

/**
 * @brief 复位观测器状态。
 *
 * @param state 输出状态。
 * @param config 观测器配置。
 * @param theta0_rad 初始电角度 rad。
 * @param omega0_rad_s 初始电角速度 rad/s。
 * @return 成功返回 true。
 *
 * 调用上下文：每次启动前复位，初始角度应取开环强拖角度。
 * 失败行为：参数非法返回 false，不修改状态。
 */
bool motor_observer_reset(motor_observer_state_t *state,
                         const motor_observer_config_t *config,
                         float theta0_rad,
                         float omega0_rad_s);

/**
 * @brief 推进一个控制周期的观测器仲裁。
 *
 * @param state 观测器状态。
 * @param config 观测器配置。
 * @param v_ab 本拍 α-β 电压 V。
 * @param i_ab 本拍 α-β 电流 A。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：参数非法或状态未复位时返回 false，并置 angle_valid 为 false。
 */
bool motor_observer_step(motor_observer_state_t *state,
                        const motor_observer_config_t *config,
                        const motor_alpha_beta_t *v_ab,
                        const motor_alpha_beta_t *i_ab,
                        float dt_s);

/**
 * @brief 查询当前角度是否可用于闭环。
 *
 * @param state 观测器状态。
 * @return true 表示角度可用且未锁存发散。
 *
 * 调用上下文：启动策略交班判据。
 * 失败行为：指针为空返回 false，保持保守判断。
 */
bool motor_observer_is_angle_valid(const motor_observer_state_t *state);

/**
 * @brief 查询是否允许交班闭环。
 *
 * @param state 观测器状态。
 * @param config 观测器配置。
 * @return true 表示角度可用且机械转速不低于闭环门限。
 *
 * 调用上下文：启动策略交班判据。
 * 失败行为：参数非法返回 false。
 */
bool motor_observer_can_close_loop(const motor_observer_state_t *state,
                                  const motor_observer_config_t *config);

/**
 * @brief 返回观测器名称。
 *
 * @param active 观测器编号。
 * @return 只读字符串；非法编号返回 "INVALID"。
 *
 * 调用上下文：日志与诊断。
 * 失败行为：非法编号不越界访问。
 */
const char *motor_observer_active_name(motor_observer_active_t active);

#endif
