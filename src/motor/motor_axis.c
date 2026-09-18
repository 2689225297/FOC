/**
 * @file motor_axis.c
 * @brief A/B 轴静态上下文和一致快照实现。
 *
 * 主要接口：motor_axis_init、motor_axis_set_parameters、motor_axis_publish_quantity、
 * motor_axis_read_snapshot。
 * 依赖关系：依赖 motor_axis.h 和 storage_parameter.h。
 * 关键安全约束：禁止动态分配；控制中断不做参数结构体的大块复制。
 */

#include "motor/motor_axis.h"

#include <string.h>

#include "storage/storage_parameter.h"

#ifndef FOC_HOST_TEST
#include "at32f403a_407.h"
#endif

/** @brief 两轴静态上下文。 */
static axis_context_t g_axis_context[MOTOR_AXIS_COUNT];

/**
 * @brief 进入短临界区。
 *
 * @return 进入前 PRIMASK；主机测试下返回 0。
 *
 * 调用上下文：快照发布和读取。
 * 失败行为：无失败路径。
 */
static uint32_t motor_axis_critical_enter(void)
{
#ifdef FOC_HOST_TEST
  return 0u;
#else
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
#endif
}

/**
 * @brief 退出短临界区。
 *
 * @param primask motor_axis_critical_enter 返回值。
 * @return 无返回值。
 *
 * 调用上下文：与进入函数成对调用。
 * 失败行为：无失败路径。
 */
static void motor_axis_critical_exit(uint32_t primask)
{
#ifndef FOC_HOST_TEST
  __set_PRIMASK(primask);
#else
  (void)primask;
#endif
}

/**
 * @brief 判断轴编号是否合法。
 *
 * @param axis 输入编号。
 * @return true 表示 AXIS_A 或 AXIS_B。
 *
 * 调用上下文：所有公开接口。
 * 失败行为：无失败路径。
 */
static bool motor_axis_valid(axis_id_t axis)
{
  return (axis == AXIS_A) || (axis == AXIS_B);
}

/**
 * @brief 初始化单轴静态上下文。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 *
 * 调用上下文：系统启动。
 * 失败行为：非法轴返回 false。
 */
bool motor_axis_init(axis_id_t axis)
{
  if (!motor_axis_valid(axis)) {
    return false;
  }

  memset(&g_axis_context[axis], 0, sizeof(g_axis_context[axis]));
  g_axis_context[axis].id = axis;
  g_axis_context[axis].initialized = true;
  g_axis_context[axis].snapshot.state = APP_STATE_POWER_ON;
  g_axis_context[axis].snapshot.mode = MOTOR_MODE_STOP;
  storage_parameter_defaults(&g_axis_context[axis].parameters);
  return true;
}

/**
 * @brief 整体替换轴参数。
 *
 * @param axis 轴编号。
 * @param parameters 候选参数。
 * @return true 表示复制成功。
 *
 * 调用上下文：停机期。
 * 失败行为：非法参数返回 false，旧参数不变。
 */
bool motor_axis_set_parameters(axis_id_t axis, const parameter_set_t *parameters)
{
  uint32_t primask;

  if (!motor_axis_valid(axis) || (parameters == 0) || !g_axis_context[axis].initialized) {
    return false;
  }

  primask = motor_axis_critical_enter();
  g_axis_context[axis].parameters = *parameters;
  g_axis_context[axis].snapshot.parameter_version = parameters->parameter_version;
  motor_axis_critical_exit(primask);
  return true;
}

/**
 * @brief 读取活动参数副本。
 *
 * @param axis 轴编号。
 * @param parameters 输出参数。
 * @return true 表示复制成功。
 *
 * 调用上下文：任务和停机期控制初始化。
 * 失败行为：非法输入返回 false。
 */
bool motor_axis_get_parameters(axis_id_t axis, parameter_set_t *parameters)
{
  uint32_t primask;

  if (!motor_axis_valid(axis) || (parameters == 0) || !g_axis_context[axis].initialized) {
    return false;
  }

  primask = motor_axis_critical_enter();
  *parameters = g_axis_context[axis].parameters;
  motor_axis_critical_exit(primask);
  return true;
}

/**
 * @brief 发布完整遥测快照。
 *
 * @param axis 轴编号。
 * @param quantities 电机量。
 * @param state 当前状态。
 * @param mode 当前模式。
 * @param fault_flags 故障位图。
 * @param timestamp_ms 当前毫秒时间。
 * @return true 表示发布成功。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：非法输入返回 false。
 */
bool motor_axis_publish_quantity(axis_id_t axis,
                                 const motor_quantity_t *quantities,
                                 app_state_t state,
                                 motor_mode_t mode,
                                 uint32_t fault_flags,
                                 uint32_t timestamp_ms)
{
  uint32_t primask;

  if (!motor_axis_valid(axis) || (quantities == 0) || !g_axis_context[axis].initialized) {
    return false;
  }

  primask = motor_axis_critical_enter();
  ++g_axis_context[axis].publish_sequence;
  g_axis_context[axis].quantities = *quantities;
  g_axis_context[axis].snapshot.sequence = g_axis_context[axis].publish_sequence;
  g_axis_context[axis].snapshot.timestamp_ms = timestamp_ms;
  g_axis_context[axis].snapshot.state = state;
  g_axis_context[axis].snapshot.mode = mode;
  g_axis_context[axis].snapshot.quantities = *quantities;
  g_axis_context[axis].snapshot.fault_flags = fault_flags;
  g_axis_context[axis].snapshot.reserved = 0u;
  motor_axis_critical_exit(primask);
  return true;
}

/**
 * @brief 读取一致遥测快照。
 *
 * @param axis 轴编号。
 * @param snapshot 输出快照。
 * @return true 表示序号前后一致。
 *
 * 调用上下文：1kHz 和通信任务。
 * 失败行为：序号变化或输入非法返回 false。
 */
bool motor_axis_read_snapshot(axis_id_t axis, telemetry_snapshot_t *snapshot)
{
  uint32_t primask;
  uint32_t before;
  uint32_t after;

  if (!motor_axis_valid(axis) || (snapshot == 0) || !g_axis_context[axis].initialized) {
    return false;
  }

  primask = motor_axis_critical_enter();
  before = g_axis_context[axis].publish_sequence;
  *snapshot = g_axis_context[axis].snapshot;
  after = g_axis_context[axis].publish_sequence;
  motor_axis_critical_exit(primask);

  return before == after;
}
