/**
 * @file motor_axis.h
 * @brief A/B 两轴静态上下文所有权接口。
 *
 * 主要接口：轴初始化、参数替换、电机量发布和一致快照读取。
 * 依赖关系：依赖 motor_types.h；实现不使用动态内存或阻塞锁。
 * 关键安全约束：参数只允许在轴停止时替换；遥测快照必须作为一个整体发布。
 */

#ifndef FOC_MOTOR_AXIS_H
#define FOC_MOTOR_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单轴静态上下文。
 *
 * 所有字段都属于同一轴；任何模块只能通过 motor_axis.c 接口读写。
 */
typedef struct {
  axis_id_t id;                       /**< 轴编号，初始化后不可变化。 */
  bool initialized;                   /**< 静态上下文初始化标志。 */
  parameter_set_t parameters;         /**< 当前生效参数，停机期整体替换。 */
  motor_quantity_t quantities;        /**< 最近一个控制周期的电机量。 */
  telemetry_snapshot_t snapshot;      /**< 一致遥测快照。 */
  uint32_t publish_sequence;          /**< 快照发布序号，发布时递增。 */
} axis_context_t;

/**
 * @brief 初始化指定轴静态上下文。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 *
 * 调用上下文：系统启动、中断使能前。
 * 失败行为：非法轴返回 false；初始参数为明确未标定的安全默认值。
 */
bool motor_axis_init(axis_id_t axis);

/**
 * @brief 替换指定轴活动参数。
 *
 * @param axis 轴编号。
 * @param parameters 候选参数。
 * @return true 表示参数已整体复制。
 *
 * 调用上下文：仅允许轴处于 IDLE、COAST 或 FAULT 时调用。
 * 失败行为：非法轴或空指针返回 false，旧参数保持不变。
 */
bool motor_axis_set_parameters(axis_id_t axis, const parameter_set_t *parameters);

/**
 * @brief 读取指定轴活动参数副本。
 *
 * @param axis 轴编号。
 * @param parameters 输出参数。
 * @return true 表示复制成功。
 *
 * 调用上下文：控制、安全和非实时任务。
 * 失败行为：非法参数返回 false，不修改输出。
 */
bool motor_axis_get_parameters(axis_id_t axis, parameter_set_t *parameters);

/**
 * @brief 发布一个完整控制周期电机量。
 *
 * @param axis 轴编号。
 * @param quantities 输入电机量。
 * @param state 当前状态。
 * @param mode 当前模式。
 * @param fault_flags 当前故障位图。
 * @param timestamp_ms 当前毫秒时间。
 * @return true 表示快照发布成功。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：非法参数返回 false；成功时先增加序号，再更新整组字段。
 */
bool motor_axis_publish_quantity(axis_id_t axis,
                                 const motor_quantity_t *quantities,
                                 app_state_t state,
                                 motor_mode_t mode,
                                 uint32_t fault_flags,
                                 uint32_t timestamp_ms);

/**
 * @brief 读取一致遥测快照。
 *
 * @param axis 轴编号。
 * @param snapshot 输出快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：1kHz 和通信任务。
 * 失败行为：非法参数或序号在复制中变化时返回 false。
 */
bool motor_axis_read_snapshot(axis_id_t axis, telemetry_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
