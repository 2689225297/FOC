/**
 * @file motor_startup.h
 * @brief A5 单轴启动策略与观测器交班接口声明（V/F、I/F、HFI 及其回退）。
 *
 * 主要接口：配置默认值与校验、状态初始化与复位、单周期推进、策略切换与查询。
 * 依赖关系：依赖 motor_math、motor_if、motor_hfi 与冻结类型 start_strategy_t；
 * 不访问寄存器、不分配内存。
 * 关键安全约束：
 *   1. HFI 为实验性启动策略，位置误差置信度持续超门限时必须自动回退开环 I/F；
 *   2. 闭环交班必须同时满足“观测器角度可用”和“开环电频率不低于交班门限”，
 *      禁止在零低速直接用观测器角度闭环；
 *   3. 交班后观测器角度连续失效超过超时门限时，必须退回开环 I/F 并累计回退次数；
 *   4. 回退次数超过上限时置 start_failed，由上层按启动失败锁存故障并安全停机。
 */

#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_hfi.h"
#include "motor/motor_if.h"
#include "motor/motor_math.h"
#include "motor/motor_types.h"

/** @brief 启动策略阶段。 */
typedef enum
{
  MOTOR_STARTUP_IDLE = 0,       /**< 未启动。 */
  MOTOR_STARTUP_ALIGN,          /**< 预定位（I/F 强拖的第一段）。 */
  MOTOR_STARTUP_OPEN_LOOP_IF,   /**< I/F 或 V/F 开环升速。 */
  MOTOR_STARTUP_OPEN_LOOP_VF,   /**< V/F 开环升速。 */
  MOTOR_STARTUP_HFI,            /**< HFI 低频位置检测。 */
  MOTOR_STARTUP_CLOSED_LOOP,    /**< 已交班给观测器闭环。 */
  MOTOR_STARTUP_FALLBACK_IF,    /**< 由 HFI 或闭环回退到 I/F。 */
  MOTOR_STARTUP_COUNT           /**< 阶段数量哨兵。 */
} motor_startup_phase_t;

/** @brief 启动策略配置。 */
typedef struct
{
  start_strategy_t strategy;        /**< 启动策略：I/F 或实验性 HFI。 */
  motor_direction_t direction;      /**< 目标旋转方向。 */
  motor_if_config_t if_config;      /**< I/F 强拖参数。 */
  motor_hfi_config_t hfi_config;    /**< HFI 注入与解调参数。 */
  float hfi_confidence_threshold;   /**< 低置信度判据门限：|HFI 位置误差| 超过该值即判为低置信度。 */
  float hfi_confidence_hold_s;      /**< 低置信度持续时间门限 s，达到即自动回退开环 I/F。 */
  float closed_loop_min_freq_hz;    /**< 允许交班闭环的最小电频率 Hz。 */
  float observer_loss_timeout_s;    /**< 交班后观测器角度失效的超时门限 s。 */
  float vf_voltage_v;               /**< V/F 在 vf_freq_end_hz 处的目标电压 V。 */
  float vf_freq_start_hz;           /**< V/F 起始电频率 Hz。 */
  float vf_freq_rate_hz_s;          /**< V/F 电频率上升率 Hz/s。 */
  float vf_freq_end_hz;             /**< V/F 结束电频率 Hz。 */
  uint32_t max_fallback_count;      /**< 允许的最大回退次数。 */
  bool use_vf_mode;                /**< true 使用 V/F，false 使用 I/F。 */
} motor_startup_config_t;

/** @brief 启动策略运行状态。 */
typedef struct
{
  motor_startup_phase_t phase;      /**< 当前阶段。 */
  start_strategy_t strategy;        /**< 当前生效策略（回退后变为 I/F）。 */
  motor_if_state_t if_state;        /**< I/F 运行状态。 */
  motor_hfi_state_t hfi_state;      /**< HFI 运行状态。 */
  float vf_freq_hz;                 /**< V/F 当前电频率 Hz。 */
  float vf_theta_rad;               /**< V/F 当前电角度 rad。 */
  float hfi_error;                  /**< 最近一拍 HFI 位置误差幅值 |sin(2Δθ)|，越大越不可信。 */
  float low_confidence_s;           /**< 低置信度累计时间 s。 */
  float observer_loss_s;            /**< 交班后观测器角度失效累计时间 s。 */
  float elapsed_s;                  /**< 启动累计时间 s。 */
  uint32_t fallback_count;          /**< 回退次数。 */
  uint32_t step_count;              /**< 已推进周期数。 */
  bool closed_loop;                 /**< 是否已交班给观测器。 */
  bool start_failed;                /**< 回退次数超限，启动失败。 */
  bool hfi_fallback;                /**< 最近一拍是否发生 HFI 回退。 */
} motor_startup_state_t;

/** @brief 启动策略单周期输出。 */
typedef struct
{
  motor_dq_t idq_ref;          /**< 开环电流指令 A（V/F 阶段为零）。 */
  motor_dq_t vdq_ref;          /**< 开环电压指令 V（仅 V/F 阶段非零）。 */
  float theta_rad;             /**< 开环电角度 rad（闭环后由观测器角度取代）。 */
  float omega_e_rad_s;         /**< 开环电角速度 rad/s。 */
  float hfi_inject_voltage_v;  /**< 注入 d 轴电压 V（仅 HFI 阶段非零）。 */
  motor_mode_t mode;           /**< 当前控制模式建议值。 */
  bool closed_loop;            /**< 是否已交班给观测器。 */
  bool hfi_fallback;           /**< 最近一拍是否发生 HFI 回退。 */
  bool start_failed;           /**< 是否启动失败。 */
} motor_startup_output_t;

/**
 * @brief 填充启动策略默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 *
 * 调用上下文：初始化前构造候选配置。
 * 失败行为：指针为空返回 false。
 */
bool motor_startup_default_config(motor_startup_config_t *config);

/**
 * @brief 校验启动策略配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 *
 * 调用上下文：初始化与参数替换。
 * 失败行为：策略枚举非法、I/F 或 HFI 配置非法、门限非有限时返回 false。
 */
bool motor_startup_validate_config(const motor_startup_config_t *config);

/**
 * @brief 复位启动策略状态并进入预定位或 HFI 检测。
 *
 * @param state 输出状态。
 * @param config 启动策略配置。
 * @return 成功返回 true。
 *
 * 调用上下文：每次启动请求。
 * 失败行为：参数非法返回 false，不修改状态。
 */
bool motor_startup_reset(motor_startup_state_t *state, const motor_startup_config_t *config);

/**
 * @brief 推进一个控制周期的启动策略。
 *
 * @param state 启动策略状态。
 * @param config 启动策略配置。
 * @param iq_sample_a 实测 q 轴电流样本 A（HFI 解调用，其他阶段无效）。
 * @param observer_ready 观测器是否允许交班闭环。
 * @param observer_valid 观测器角度当前是否有效（交班后失效判据用）。
 * @param dt_s 控制周期 s。
 * @param output 单周期输出。
 * @return 成功返回 true。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：参数非法返回 false，不修改状态与输出。
 */
bool motor_startup_step(motor_startup_state_t *state,
                       const motor_startup_config_t *config,
                       float iq_sample_a,
                       bool observer_ready,
                       bool observer_valid,
                       float dt_s,
                       motor_startup_output_t *output);

/**
 * @brief 返回启动策略阶段名称。
 *
 * @param phase 阶段编号。
 * @return 只读字符串；非法阶段返回 "INVALID"。
 *
 * 调用上下文：日志与诊断。
 * 失败行为：非法阶段不越界访问。
 */
const char *motor_startup_phase_name(motor_startup_phase_t phase);

#endif
