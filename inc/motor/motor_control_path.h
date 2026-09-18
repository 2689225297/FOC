/**
 * @file motor_control_path.h
 * @brief A5 单轴控制路径编排接口声明（启动策略 + 观测器仲裁 + 电流环 + 安全停机）。
 *
 * 主要接口：配置默认值与校验、初始化、启动、单周期推进、安全停机与状态查询。
 * 依赖关系：依赖 motor_control、motor_startup、motor_observer 三个产品级算法模块
 * 与冻结的电机域类型；不访问寄存器、不分配内存、不调用 BSP。
 *
 * 单周期编排顺序：
 *   1. Clarke 得到 α-β 电流；
 *   2. 观测器仲裁（EKF 主观测 / SMO 回退 / 磁链监测），交叉校验失败即锁存发散；
 *   3. 启动策略推进（V/F、I/F、HFI 及 HFI 低置信度回退 I/F、闭环交班与失效回退）；
 *   4. 按启动策略输出选择角度源：开环阶段用强拖角，闭环后用观测器角；
 *   5. V/F 阶段走电压指令通路（不经过 PI），I/F 与闭环阶段走电流环通路；
 *   6. 任一环节失败、观测器发散、启动失败或限幅异常时，统一进入安全停机并上报故障位。
 *
 * 关键安全约束：
 *   1. 配置校验不通过（参数未标定或未版本化）时禁止初始化与启动；
 *   2. 观测器角度无效时不得闭环，启动策略必须继续开环或回退 I/F；
 *   3. 观测器发散、启动重试耗尽必须锁存 FAULT_OBSERVER_DIVERGED / FAULT_START_FAILED
 *      并进入安全停机，安全停机后 step 拒绝恢复输出；
 *   4. 本模块不直接驱动功率级，占空比只写入状态供上层下发，下发前必须经
 *      motor_control_duty_is_safe 与 BSP 安全门控检查。
 */

#ifndef MOTOR_CONTROL_PATH_H
#define MOTOR_CONTROL_PATH_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_control.h"
#include "motor/motor_observer.h"
#include "motor/motor_startup.h"
#include "motor/motor_types.h"

/** @brief 单轴控制路径配置。 */
typedef struct
{
  motor_control_config_t control;   /**< 电流环与 SVPWM 配置。 */
  motor_startup_config_t startup;   /**< 启动策略配置。 */
  motor_observer_config_t observer; /**< 观测器仲裁配置。 */
} motor_control_path_config_t;

/** @brief 单轴控制路径状态（供遥测、判据与故障处理读取）。 */
typedef struct
{
  motor_startup_phase_t phase;      /**< 启动策略当前阶段。 */
  motor_observer_active_t observer; /**< 当前角度源。 */
  motor_mode_t mode;                /**< 当前控制模式。 */
  motor_quantity_t quantities;      /**< 本周期电机量（机械速度为观测器估计值）。 */
  float duty[3];                    /**< 三相上桥臂占空比。 */
  float modulation_index;           /**< 归一化调制比。 */
  float theta_used_rad;             /**< 本拍实际使用的电角度 rad。 */
  float observer_speed_rpm;         /**< 观测器机械转速估计 rpm。 */
  float smo_speed_rpm;              /**< SMO 机械转速估计 rpm。 */
  float v_limit_v;                  /**< 生效电压矢量限幅 V。 */
  float hfi_error;                  /**< HFI 位置误差幅值（非 HFI 阶段为 0），越大越不可信。 */
  float angle_error_rad;            /**< EKF 与 SMO 角度差 rad。 */
  uint32_t fault_flags;             /**< 控制路径判定的故障位图。 */
  uint32_t limit_count;             /**< 电压限幅累计次数。 */
  uint32_t fallback_count;          /**< 启动回退累计次数。 */
  bool closed_loop;                 /**< 是否已交班闭环。 */
  bool start_failed;                /**< 启动是否已判定失败。 */
  bool observer_diverged;           /**< 观测器是否已锁存发散。 */
  bool flux_monitor_ok;             /**< 磁链监测结论是否可用。 */
  bool limited;                     /**< 最近一拍是否发生电压限幅。 */
  bool duty_safe;                   /**< 最近一拍占空比是否通过合法性检查。 */
  bool stopped;                     /**< 是否处于安全停机态。 */
} motor_control_path_status_t;

/**
 * @brief 填充控制路径默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 *
 * 调用上下文：初始化前构造候选配置。
 * 失败行为：指针为空返回 false；默认配置各子模块 parameters_valid 均为 false，
 * 控制路径在显式写入已归档参数前不可运行。
 */
bool motor_control_path_default_config(motor_control_path_config_t *config);

/**
 * @brief 校验控制路径配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 *
 * 调用上下文：初始化、启动与运行期自检。
 * 失败行为：任一子模块配置非法或参数未版本化时返回 false。
 */
bool motor_control_path_validate_config(const motor_control_path_config_t *config);

/**
 * @brief 构造指定轴控制路径（不进入运行态）。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @return 成功返回 true。
 *
 * 调用上下文：系统初始化或停机期参数替换。
 * 失败行为：运行态、轴非法或配置非法时返回 false，旧状态保持不变。
 */
bool motor_control_path_init(axis_id_t axis, const motor_control_path_config_t *config);

/**
 * @brief 启动指定轴控制路径：复位电流环、启动策略与观测器。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @return 成功返回 true。
 *
 * 调用上下文：收到启动命令且已完成停机条件检查后。
 * 失败行为：未初始化或配置非法时返回 false，并保持安全停机。
 */
bool motor_control_path_start(axis_id_t axis, const motor_control_path_config_t *config);

/**
 * @brief 推进一个控制周期的单轴控制路径。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @param i_abc 三相实测电流 A。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：参数非法、安全停机态或任一环节失败时返回 false，并保持占空比为零
 * 电压中点、置 fault_flags 后由上层执行安全停机与故障锁存。
 */
bool motor_control_path_step(axis_id_t axis,
                            const motor_control_path_config_t *config,
                            const motor_abc_t *i_abc,
                            float dt_s);

/**
 * @brief 立即进入安全停机并锁存停机请求。
 *
 * @param axis 轴编号。
 * @return true 表示已进入安全停机。
 *
 * 调用上下文：故障路径、停机命令与状态机非运行态。
 * 失败行为：轴非法返回 false；停机后 step 不再输出，必须重新启动才能恢复。
 */
bool motor_control_path_safe_stop(axis_id_t axis);

/**
 * @brief 读取控制路径状态副本。
 *
 * @param axis 轴编号。
 * @param status 输出状态。
 * @return 成功返回 true。
 *
 * 调用上下文：遥测、判据与故障处理。
 * 失败行为：轴非法、指针为空或未初始化时返回 false。
 */
bool motor_control_path_get_status(axis_id_t axis, motor_control_path_status_t *status);

/**
 * @brief 查询控制路径是否处于运行态。
 *
 * @param axis 轴编号。
 * @return true 表示已启动且未安全停机。
 *
 * 调用上下文：上层调度与安全判据。
 * 失败行为：轴非法返回 false。
 */
bool motor_control_path_is_active(axis_id_t axis);

#endif
