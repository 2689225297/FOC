/**
 * @file motor_control.h
 * @brief A5 单轴电流环控制路径接口声明（Clarke/Park、Id/Iq PI、解耦前馈、限幅、SVPWM）。
 *
 * 主要接口：配置默认值与校验、状态初始化与复位、电流指令设置、单周期电流环运算、
 * 安全停机与占空比合法性检查。
 * 依赖关系：依赖 motor_math、motor_pi、motor_svpwm 与冻结的电机域类型；
 * 不访问寄存器、不分配内存、不调用 BSP，可直接在主机单元测试中运行。
 * 关键安全约束：
 *   1. parameters_valid 为 false（参数未标定或未版本化）时校验失败，motor_control_step
 *      返回 false 且不更新占空比，调用方必须保持功率输出关闭；
 *   2. 电压矢量限幅取“配置限幅”与“母线线性区 Udc/√3”中的较小值，限幅后不得再放大，
 *      每次限幅都置 limited 并累计 limit_count，供上层判据与遥测使用；
 *   3. 任何非有限输入（NaN/Inf）都被拒绝，禁止把非有限值传播为 PWM 比较值；
 *   4. 高频注入电压只作为 d 轴叠加项，必须先与 PI 输出叠加、再经电压矢量限幅统一
 *      约束，禁止绕过限幅直接下发；
 *   5. 进入故障或停机后必须调用 motor_control_safe_stop；停机状态下 step 直接返回
 *      false，占空比保持零电压中点值，由 BSP 关闭功率输出。
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_math.h"
#include "motor/motor_pi.h"
#include "motor/motor_svpwm.h"

/**
 * @brief 电流环配置。
 *
 * 电机电气参数必须来自已归档的版本化标定记录；parameters_valid 未置位时整条
 * 控制路径拒绝运行。
 */
typedef struct
{
  motor_motor_params_t motor;   /**< 已标定的 Rs、Ld、Lq 与磁链。 */
  float kp_d;                   /**< d 轴电流环比例增益 V/A。 */
  float ki_d;                   /**< d 轴电流环积分增益 V/(A·s)。 */
  float kp_q;                   /**< q 轴电流环比例增益 V/A。 */
  float ki_q;                   /**< q 轴电流环积分增益 V/(A·s)。 */
  float voltage_limit_v;        /**< 电压矢量限幅 V；<= 0 表示只受母线线性区约束。 */
  float current_limit_a;        /**< 电流指令幅值限幅 A，必须大于 0。 */
  bool enable_decoupling;       /**< 是否叠加解耦前馈电压。 */
  bool parameters_valid;        /**< 参数是否来自已归档的版本化标定记录。 */
  motor_svpwm_config_t svpwm;   /**< SVPWM 配置（载波、母线、死区、最小脉宽）。 */
} motor_control_config_t;

/**
 * @brief 电流环单轴运行状态。
 *
 * 所有字段由本模块维护，调用方只读；duty 只在 step 成功且未安全停机时有效。
 */
typedef struct
{
  motor_pi_t pi_d;              /**< d 轴 PI 调节器。 */
  motor_pi_t pi_q;              /**< q 轴 PI 调节器。 */
  motor_alpha_beta_t i_ab;      /**< 最近一拍 α-β 电流 A。 */
  motor_dq_t idq;               /**< 最近一拍 d-q 电流 A。 */
  motor_dq_t idq_ref;           /**< 当前 d-q 电流指令 A（已限幅）。 */
  motor_dq_t vdq_ff;            /**< 最近一拍解耦前馈电压 V。 */
  float vd_inject_v;            /**< 高频注入 d 轴电压 V，与 PI 输出叠加后统一限幅。 */
  motor_dq_t vdq;               /**< 最近一拍限幅后的 d-q 电压指令 V。 */
  motor_alpha_beta_t v_ab;      /**< 最近一拍 α-β 电压指令 V。 */
  motor_svpwm_result_t modulation; /**< 最近一拍 SVPWM 求解结果。 */
  float duty[3];                /**< 三相上桥臂占空比，安全停机时为 0.5。 */
  float v_limit_v;              /**< 本配置下生效的电压矢量限幅 V。 */
  uint32_t step_count;          /**< 已执行的电流环周期数。 */
  uint32_t limit_count;         /**< 电压矢量限幅累计次数。 */
  bool reference_limited;       /**< 最近一次电流指令是否被限幅。 */
  bool limited;                 /**< 最近一拍是否发生电压限幅。 */
  bool stopped;                 /**< 是否处于安全停机态（step 拒绝运行）。 */
} motor_control_state_t;

/**
 * @brief 填充电流环默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 *
 * 调用上下文：初始化前构造候选配置。
 * 失败行为：指针为空返回 false。
 */
bool motor_control_default_config(motor_control_config_t *config);

/**
 * @brief 校验电流环配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 *
 * 调用上下文：初始化与参数替换。
 * 失败行为：parameters_valid 未置位、电机参数非法或 SVPWM 配置非法时返回 false。
 */
bool motor_control_validate_config(const motor_control_config_t *config);

/**
 * @brief 返回本配置下生效的电压矢量限幅值。
 *
 * @param config 电流环配置。
 * @return 限幅值 V；配置非法返回 0，调用方必须按零输出处理。
 *
 * 调用上下文：初始化、限幅检查与遥测。
 * 失败行为：配置为空返回 0。
 */
float motor_control_voltage_limit(const motor_control_config_t *config);

/**
 * @brief 初始化电流环状态与 PI 调节器。
 *
 * @param state 输出状态。
 * @param config 电流环配置。
 * @return 成功返回 true。
 *
 * 调用上下文：轴停机、参数校验通过后的初始化。
 * 失败行为：参数非法返回 false，不修改状态。
 */
bool motor_control_init(motor_control_state_t *state, const motor_control_config_t *config);

/**
 * @brief 复位运行状态（保留 PI 增益，清零积分、计数与限幅标志）。
 *
 * @param state 电流环状态。
 * @return 无返回值。
 *
 * 调用上下文：每次启动前复位。
 * 失败行为：指针为空时不做任何操作。
 */
void motor_control_reset(motor_control_state_t *state);

/**
 * @brief 设置 d-q 电流指令并解除安全停机标志。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param idq_ref 指令电流 A。
 * @return 成功返回 true。
 *
 * 调用上下文：启动策略与闭环外环；调用即表示控制路径转入运行态，调用方必须已
 * 完成停机条件检查。
 * 失败行为：参数非法或非有限值返回 false，原指令与停机标志保持不变。
 */
bool motor_control_set_current_reference(motor_control_state_t *state,
                                        const motor_control_config_t *config,
                                        const motor_dq_t *idq_ref);

/**
 * @brief 设置 d 轴高频注入电压（与电流环 PI 输出叠加）。
 *
 * @param state 电流环状态。
 * @param vd_inject_v 注入电压 V；0 表示关闭注入。
 * @return 成功返回 true。
 *
 * 调用上下文：零低速 HFI 启动阶段的 20kHz 控制中断，每拍按注入极性更新。
 * 失败行为：指针为空或注入值非有限时返回 false，原注入值保持不变。注入值本身
 * 允许为负（方波极性），但必经电压矢量限幅，不得直接下发。
 */
bool motor_control_set_voltage_injection(motor_control_state_t *state, float vd_inject_v);

/**
 * @brief 执行一拍电流环运算并更新占空比。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param i_abc 三相实测电流 A。
 * @param theta_rad 本拍使用的电角度 rad（开环角或观测器角）。
 * @param omega_e_rad_s 本拍使用的电角速度 rad/s（解耦前馈用）。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：安全停机态、参数非法或任一输入非有限时返回 false，且不更新占空比。
 */
bool motor_control_step(motor_control_state_t *state,
                       const motor_control_config_t *config,
                       const motor_abc_t *i_abc,
                       float theta_rad,
                       float omega_e_rad_s,
                       float dt_s);

/**
 * @brief 执行一拍开环电压（V/F）运算并更新占空比。
 *
 * 与 motor_control_step 的区别：跳过 PI 与解耦前馈，直接使用电压指令，仅保留
 * 电压矢量限幅与 SVPWM 约束。实测电流仍会换算为 α-β 与 d-q 供遥测和故障判据
 * 使用，但不参与电压生成。
 *
 * @param state 电流环状态。
 * @param config 电流环配置。
 * @param i_abc 三相实测电流 A。
 * @param vdq_ref 开环 d-q 电压指令 V。
 * @param theta_rad 本拍电角度 rad。
 * @return 成功返回 true。
 *
 * 调用上下文：20kHz 控制中断的 V/F 开环阶段。
 * 失败行为：安全停机态、参数非法或任一输入非有限时返回 false，且不更新占空比。
 */
bool motor_control_step_voltage(motor_control_state_t *state,
                               const motor_control_config_t *config,
                               const motor_abc_t *i_abc,
                               const motor_dq_t *vdq_ref,
                               float theta_rad);

/**
 * @brief 进入安全停机：清零积分与限幅标志，占空比置零电压中点。
 *
 * @param state 电流环状态。
 * @return true 表示已进入安全停机。
 *
 * 调用上下文：故障、停机请求和状态机非运行态。
 * 失败行为：指针为空返回 false。
 */
bool motor_control_safe_stop(motor_control_state_t *state);

/**
 * @brief 查询是否处于安全停机态。
 *
 * @param state 电流环状态。
 * @return true 表示已安全停机（step 拒绝运行）。
 *
 * 调用上下文：故障路径与遥测。
 * 失败行为：指针为空返回 true，保持保守判断。
 */
bool motor_control_is_stopped(const motor_control_state_t *state);

/**
 * @brief 检查当前三相占空比是否为可下发的合法值。
 *
 * @param state 电流环状态。
 * @return true 表示三路占空比均为有限值且落在 0..1。
 *
 * 调用上下文：下发 PWM 前的最后一道软件检查。
 * 失败行为：指针为空或任一占空比非法返回 false。
 */
bool motor_control_duty_is_safe(const motor_control_state_t *state);

#endif
