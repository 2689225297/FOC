/**
 * @file app_control_core.h
 * @brief 双轴 20kHz 控制核心：调度、限速、超速与 CAN 超时转矩归零接口。
 *
 * 主要接口：每轴 init/configure/start/stop/step、转速治理、CAN 超时归零、
 *          停机写入限制和恢复。
 * 依赖关系：依赖 motor_control_path.h 与 motor_types.h，纯逻辑可在主机测试。
 * 关键安全约束：单轴故障锁存只影响该轴；限速点 7000rpm 进入限速停机，
 *              7500rpm 锁存超速故障；参数写入只允许在全部轴停机时进行。
 */

#ifndef FOC_APP_CONTROL_CORE_H
#define FOC_APP_CONTROL_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_control_path.h"
#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单轴控制核心状态。
 *
 * 取值范围：STOPPED 允许配置与启动；RUNNING 处于 20kHz 调度；
 *          LIMIT_ENGAGED 限速停机可恢复；FAULT_LATCHED 锁存故障需恢复流程；
 *          TORQUE_ZEROED CAN 超时转矩归零可恢复。
 */
typedef enum {
  APP_CONTROL_CORE_STOPPED = 0,  /**< 已停机且可配置。 */
  APP_CONTROL_CORE_RUNNING,      /**< 正在 20kHz 控制调度。 */
  APP_CONTROL_CORE_LIMIT_ENGAGED,/**< 达到限速点，已安全停机，可恢复。 */
  APP_CONTROL_CORE_FAULT_LATCHED,/**< 锁存故障，需人工恢复。 */
  APP_CONTROL_CORE_TORQUE_ZEROED /**< CAN 超时转矩归零，已安全停机，可恢复。 */
} app_control_core_state_t;

/**
 * @brief 单轴控制核心状态快照。
 */
typedef struct {
  app_control_core_state_t state;  /**< 核心状态。 */
  motor_control_path_status_t path;/**< 控制路径状态。 */
  uint32_t latched_fault_flags;    /**< 锁存故障位图，见 fault_code_t。 */
  bool torque_zeroed;              /**< CAN 超时归零标志，恢复后清除。 */
} app_control_core_status_t;

/**
 * @brief 初始化指定轴控制核心。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 *
 * 调用上下文：系统初始化、中断使能前。
 * 失败行为：非法轴返回 false。
 */
bool app_control_core_init(axis_id_t axis);

/**
 * @brief 在停机期替换控制路径配置。
 *
 * @param axis 轴编号。
 * @param config 新配置，必须通过 motor_control_path_validate_config。
 * @return true 表示替换成功。
 *
 * 调用上下文：1kHz 参数管理任务，仅在全部轴停机时调用。
 * 失败行为：配置非法或轴处于 RUNNING 时返回 false，保持旧配置。
 */
bool app_control_core_configure(axis_id_t axis, const motor_control_path_config_t *config);

/**
 * @brief 启动指定轴控制路径。
 *
 * @param axis 轴编号。
 * @param parameters 当前参数，必须 storage_parameter_is_operational 通过。
 * @return true 表示启动成功。
 *
 * 调用上下文：状态机进入启动事件后。
 * 失败行为：参数未标定、核心未配置、已锁存故障或已在运行时返回 false。
 */
bool app_control_core_start(axis_id_t axis, const parameter_set_t *parameters);

/**
 * @brief 正常安全停机，不锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示停机请求被接受。
 *
 * 调用上下文：状态机停止事件。
 * 失败行为：非法轴返回 false。
 */
bool app_control_core_stop(axis_id_t axis);

/**
 * @brief 执行一拍 20kHz 控制。
 *
 * @param axis 轴编号。
 * @param i_abc 三相电流反馈，可为 0 表示无电流环数据。
 * @param dt_s 控制周期，单位 s。
 * @return true 表示本拍控制成功。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：核心不在 RUNNING 时拒绝执行并返回 false；
 *           控制路径失败时锁存 FAULT_INTERNAL 并安全停机，仅影响本轴。
 */
bool app_control_core_step(axis_id_t axis, const motor_abc_t *i_abc, float dt_s);

/**
 * @brief 转速治理：限速停机或超速故障锁存。
 *
 * @param axis 轴编号。
 * @param speed_rpm 当前机械转速，单位 rpm。
 * @return 无返回值。
 *
 * 调用上下文：20kHz 控制中断，在 step 之后调用。
 * 失败行为：转速输入无效时按超速故障保守处理；只影响本轴。
 */
void app_control_core_speed_govern(axis_id_t axis, float speed_rpm);

/**
 * @brief CAN 超时转矩归零。
 *
 * @param axis 轴编号。
 * @return 无返回值。
 *
 * 调用上下文：20kHz 控制中断，在 CAN 超时检查之后调用。
 * 失败行为：只在 RUNNING 时动作；不锁存故障，允许后续 CAN 恢复后重新启动。
 */
void app_control_core_can_timeout_zero(axis_id_t axis);

/**
 * @brief 读取指定轴控制核心状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：诊断与遥测。
 * 失败行为：指针为空或非法轴返回 false。
 */
bool app_control_core_get_status(axis_id_t axis, app_control_core_status_t *status);

/**
 * @brief 查询指定轴是否锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示存在锁存故障或非法轴。
 *
 * 调用上下文：状态机与诊断。
 * 失败行为：非法轴返回 true，保持保守。
 */
bool app_control_core_has_fault(axis_id_t axis);

/**
 * @brief 查询是否允许参数事务写入。
 *
 * @param axis 轴编号。
 * @return true 表示该轴已停机且无锁存故障，允许写入。
 *
 * 调用上下文：1kHz 参数管理任务。
 * 失败行为：非法轴返回 false。
 */
bool app_control_core_can_write_parameters(axis_id_t axis);

/**
 * @brief 恢复流程：清除锁存故障并回到 STOPPED。
 *
 * @param axis 轴编号。
 * @return true 表示恢复成功。
 *
 * 调用上下文：状态机 RECOVERY_CONFIRMED 后。
 * 失败行为：未锁存故障时返回 false；只影响本轴。
 */
bool app_control_core_recover(axis_id_t axis);

#ifdef __cplusplus
}
#endif

#endif
