/**
 * @file app_control_core.c
 * @brief 双轴 20kHz 控制核心实现。
 *
 * 每轴独立持有控制路径状态；单轴锁存故障、限速、CAN 超时归零只作用于本轴，
 * 不修改另一轴状态。参数写入限制按"本轴停机且无锁存故障"判定。
 */

#include "app/app_control_core.h"

#include <string.h>

#include "safety/safety_speed.h"
#include "storage/storage_parameter.h"

/** @brief 每轴控制核心上下文。 */
typedef struct {
  motor_control_path_config_t config;   /**< 控制路径配置副本。 */
  parameter_set_t parameters;           /**< 限速/超速参数副本。 */
  app_control_core_state_t state;       /**< 核心状态。 */
  uint32_t latched_fault_flags;         /**< 锁存故障位图。 */
  bool torque_zeroed;                   /**< CAN 超时归零标志。 */
  bool configured;                      /**< 配置是否已就绪。 */
  bool initialized;                     /**< 是否已初始化。 */
} app_control_core_context_t;

/** @brief 全部轴上下文，静态分配。 */
static app_control_core_context_t g_contexts[AXIS_COUNT];

/**
 * @brief 获取指定轴上下文指针。
 *
 * @param axis 轴编号。
 * @return 上下文指针；非法轴返回 NULL。
 */
static app_control_core_context_t *app_control_core_get(axis_id_t axis)
{
  if ((axis != AXIS_A) && (axis != AXIS_B)) {
    return 0;
  }

  return &g_contexts[(uint32_t)axis];
}

/**
 * @brief 初始化指定轴控制核心。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 */
bool app_control_core_init(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if (context == 0) {
    return false;
  }

  memset(context, 0, sizeof(*context));
  context->state = APP_CONTROL_CORE_STOPPED;
  context->initialized = true;
  return true;
}

/**
 * @brief 在停机期替换控制路径配置。
 *
 * @param axis 轴编号。
 * @param config 新配置。
 * @return true 表示替换成功。
 */
bool app_control_core_configure(axis_id_t axis, const motor_control_path_config_t *config)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || (config == 0) || !context->initialized) {
    return false;
  }

  if (context->state == APP_CONTROL_CORE_RUNNING) {
    return false;
  }

  if (!motor_control_path_validate_config(config)) {
    return false;
  }

  context->config = *config;

  /* 预热控制路径，保证后续 start 立即可用。 */
  if (!motor_control_path_init(axis, &context->config)) {
    return false;
  }

  context->configured = true;
  return true;
}

/**
 * @brief 启动指定轴控制路径。
 *
 * @param axis 轴编号。
 * @param parameters 当前参数。
 * @return true 表示启动成功。
 */
bool app_control_core_start(axis_id_t axis, const parameter_set_t *parameters)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || (parameters == 0) || !context->initialized) {
    return false;
  }

  if (!context->configured) {
    return false;
  }

  if (context->state == APP_CONTROL_CORE_FAULT_LATCHED) {
    return false;
  }

  if (context->state == APP_CONTROL_CORE_RUNNING) {
    return false;
  }

  if (!storage_parameter_is_operational(parameters)) {
    return false;
  }

  context->parameters = *parameters;
  context->torque_zeroed = false;

  if (!motor_control_path_start(axis, &context->config)) {
    return false;
  }

  context->state = APP_CONTROL_CORE_RUNNING;
  return true;
}

/**
 * @brief 正常安全停机，不锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示停机请求被接受。
 */
bool app_control_core_stop(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || !context->initialized) {
    return false;
  }

  if (context->state == APP_CONTROL_CORE_RUNNING) {
    (void)motor_control_path_safe_stop(axis);
  }

  context->state = APP_CONTROL_CORE_STOPPED;
  return true;
}

/**
 * @brief 执行一拍 20kHz 控制。
 *
 * @param axis 轴编号。
 * @param i_abc 三相电流反馈，可为 0。
 * @param dt_s 控制周期，单位 s。
 * @return true 表示本拍控制成功。
 */
bool app_control_core_step(axis_id_t axis, const motor_abc_t *i_abc, float dt_s)
{
  app_control_core_context_t *context = app_control_core_get(axis);
  motor_abc_t zero;

  if ((context == 0) || !context->initialized) {
    return false;
  }

  if (context->state != APP_CONTROL_CORE_RUNNING) {
    return false;
  }

  if (i_abc == 0) {
    memset(&zero, 0, sizeof(zero));
    i_abc = &zero;
  }

  if (!motor_control_path_step(axis, &context->config, i_abc, dt_s)) {
    (void)motor_control_path_safe_stop(axis);
    context->latched_fault_flags |= FAULT_INTERNAL;
    context->state = APP_CONTROL_CORE_FAULT_LATCHED;
    return false;
  }

  return true;
}

/**
 * @brief 转速治理：限速停机或超速故障锁存。
 *
 * @param axis 轴编号。
 * @param speed_rpm 当前机械转速，单位 rpm。
 * @return 无返回值。
 */
void app_control_core_speed_govern(axis_id_t axis, float speed_rpm)
{
  app_control_core_context_t *context = app_control_core_get(axis);
  safety_speed_verdict_t verdict;
  bool valid;

  if ((context == 0) || !context->initialized) {
    return;
  }

  if (context->state != APP_CONTROL_CORE_RUNNING) {
    return;
  }

  valid = safety_speed_check(speed_rpm,
                             context->parameters.max_speed_rpm,
                             context->parameters.over_speed_fault_rpm,
                             &verdict);

  if (!valid || (verdict == SAFETY_SPEED_FAULT)) {
    (void)motor_control_path_safe_stop(axis);
    context->latched_fault_flags |= FAULT_OVER_SPEED;
    context->state = APP_CONTROL_CORE_FAULT_LATCHED;
    return;
  }

  if (verdict == SAFETY_SPEED_LIMIT) {
    (void)motor_control_path_safe_stop(axis);
    context->state = APP_CONTROL_CORE_LIMIT_ENGAGED;
  }
}

/**
 * @brief CAN 超时转矩归零。
 *
 * @param axis 轴编号。
 * @return 无返回值。
 */
void app_control_core_can_timeout_zero(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || !context->initialized) {
    return;
  }

  if (context->state != APP_CONTROL_CORE_RUNNING) {
    return;
  }

  (void)motor_control_path_safe_stop(axis);
  context->state = APP_CONTROL_CORE_TORQUE_ZEROED;
  context->torque_zeroed = true;
}

/**
 * @brief 读取指定轴控制核心状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出快照。
 * @return true 表示读取成功。
 */
bool app_control_core_get_status(axis_id_t axis, app_control_core_status_t *status)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || (status == 0) || !context->initialized) {
    return false;
  }

  memset(status, 0, sizeof(*status));
  status->state = context->state;
  status->latched_fault_flags = context->latched_fault_flags;
  status->torque_zeroed = context->torque_zeroed;
  (void)motor_control_path_get_status(axis, &status->path);
  return true;
}

/**
 * @brief 查询指定轴是否锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示存在锁存故障或非法轴。
 */
bool app_control_core_has_fault(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || !context->initialized) {
    return true;
  }

  return context->latched_fault_flags != 0u;
}

/**
 * @brief 查询是否允许参数事务写入。
 *
 * @param axis 轴编号。
 * @return true 表示该轴已停机且无锁存故障。
 */
bool app_control_core_can_write_parameters(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || !context->initialized) {
    return false;
  }

  return (context->state == APP_CONTROL_CORE_STOPPED) &&
         (context->latched_fault_flags == 0u);
}

/**
 * @brief 恢复流程：清除锁存故障并回到 STOPPED。
 *
 * @param axis 轴编号。
 * @return true 表示恢复成功。
 */
bool app_control_core_recover(axis_id_t axis)
{
  app_control_core_context_t *context = app_control_core_get(axis);

  if ((context == 0) || !context->initialized) {
    return false;
  }

  if (context->latched_fault_flags == 0u) {
    return false;
  }

  (void)motor_control_path_safe_stop(axis);
  context->latched_fault_flags = 0u;
  context->torque_zeroed = false;
  context->state = APP_CONTROL_CORE_STOPPED;
  return true;
}
