/**
 * @file safety_fault.c
 * @brief 故障锁存和固定容量事件队列实现。
 *
 * 主要接口：safety_init、safety_request_fault、safety_fast_fault、safety_event_pop。
 * 依赖关系：依赖 safety.h 和 BSP 安全输出接口。
 * 关键安全约束：锁存位先于诊断事件更新；队列满不能阻止或延迟硬件安全输出。
 */

#include "safety/safety.h"

#include <string.h>

#include "bsp/bsp_safe_outputs.h"

#ifndef FOC_HOST_TEST
#include "at32f403a_407.h"
#endif

/** @brief 总线故障在内部数组中的索引。 */
#define SAFETY_BUS_INDEX ((uint32_t)AXIS_COUNT)

/** @brief 内部故障域数量，包含两轴和总线。 */
#define SAFETY_DOMAIN_COUNT ((uint32_t)AXIS_COUNT + 1u)

/** @brief 每轴锁存故障位图。 */
static volatile uint32_t g_fault_flags[SAFETY_DOMAIN_COUNT];

/** @brief 故障事件环形队列。 */
static volatile safety_fault_record_t g_fault_queue[SAFETY_FAULT_QUEUE_CAPACITY];

/** @brief 环形队列读索引；只在任务上下文修改。 */
static volatile uint32_t g_queue_head;

/** @brief 环形队列写索引；中断和任务都可能修改。 */
static volatile uint32_t g_queue_tail;

/** @brief 队列满时丢弃的事件数。 */
static volatile uint32_t g_dropped_events;

/** @brief 故障事件序号。 */
static volatile uint16_t g_event_sequence;

/**
 * @brief 进入短临界区。
 *
 * @return 进入前的 PRIMASK；主机测试下固定返回 0。
 *
 * 调用上下文：中断和任务。
 * 失败行为：无失败路径。
 */
static uint32_t safety_critical_enter(void)
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
 * @param primask safety_critical_enter 返回的 PRIMASK。
 * @return 无返回值。
 *
 * 调用上下文：与 safety_critical_enter 成对调用。
 * 失败行为：无失败路径。
 */
static void safety_critical_exit(uint32_t primask)
{
#ifndef FOC_HOST_TEST
  __set_PRIMASK(primask);
#else
  (void)primask;
#endif
}

/**
 * @brief 把逻辑故障域转换为内部索引。
 *
 * @param axis 轴编号或 AXIS_COUNT。
 * @return 内部索引；非法输入返回 SAFETY_DOMAIN_COUNT。
 *
 * 调用上下文：故障请求和查询。
 * 失败行为：非法输入返回越界哨兵，调用方必须拒绝。
 */
static uint32_t safety_domain_index(axis_id_t axis)
{
  if ((axis == AXIS_A) || (axis == AXIS_B) || (axis == AXIS_COUNT)) {
    return (uint32_t)axis;
  }

  return SAFETY_DOMAIN_COUNT;
}

/**
 * @brief 初始化故障位图、队列和计数。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统启动、中断使能前。
 * 失败行为：无失败路径。
 */
void safety_init(void)
{
  uint32_t index;

  for (index = 0u; index < SAFETY_DOMAIN_COUNT; ++index) {
    g_fault_flags[index] = 0u;
  }

  memset((void *)g_fault_queue, 0, sizeof(g_fault_queue));
  g_queue_head = 0u;
  g_queue_tail = 0u;
  g_dropped_events = 0u;
  g_event_sequence = 0u;
}

/**
 * @brief 请求并锁存故障。
 *
 * @param axis 轴编号或总线哨兵。
 * @param fault_code 故障位。
 * @param severity 故障严重度。
 * @param context 故障上下文，可为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：中断和任务。
 * 失败行为：输入非法或队列满时仍保留锁存位并更新丢弃计数。
 */
void safety_request_fault(axis_id_t axis,
                          uint32_t fault_code,
                          fault_severity_t severity,
                          const fault_context_t *context)
{
  uint32_t primask;
  uint32_t domain = safety_domain_index(axis);

  if ((domain >= SAFETY_DOMAIN_COUNT) || (fault_code == 0u)) {
    return;
  }

  primask = safety_critical_enter();
  g_fault_flags[domain] |= fault_code;
  safety_critical_exit(primask);

  primask = safety_critical_enter();
  {
    uint32_t next_tail = (g_queue_tail + 1u) % SAFETY_FAULT_QUEUE_CAPACITY;

    if (next_tail == g_queue_head) {
      g_dropped_events++;
    } else {
      g_fault_queue[g_queue_tail].axis = axis;
      g_fault_queue[g_queue_tail].fault_code = fault_code;
      g_fault_queue[g_queue_tail].severity = severity;
      if (context != 0) {
        g_fault_queue[g_queue_tail].context = *context;
      } else {
        memset((void *)&g_fault_queue[g_queue_tail].context, 0,
               sizeof(g_fault_queue[g_queue_tail].context));
      }
      g_fault_queue[g_queue_tail].sequence = g_event_sequence++;
      g_fault_queue[g_queue_tail].reserved = 0u;
      g_queue_tail = next_tail;
    }
  }
  safety_critical_exit(primask);
}

/**
 * @brief 执行最低延迟安全动作并锁存故障。
 *
 * @param fault_code 故障位。
 * @param context 故障上下文，可为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：最高优先级中断。
 * 失败行为：bsp_safe_outputs 必须先于任何可能阻塞或较慢的诊断操作执行。
 */
void safety_fast_fault(uint32_t fault_code, const fault_context_t *context)
{
  bsp_safe_outputs();
  safety_request_fault(AXIS_B, fault_code, FAULT_SEVERITY_HARDWARE, context);
}

/**
 * @brief 执行总线级安全动作并锁存故障。
 *
 * @param fault_code 故障位。
 * @param context 故障上下文，可为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：NMI、HardFault 和总线安全异常。
 * 失败行为：先关闭双轴输出；故障队列满时仍保留总线锁存位。
 */
void safety_fast_bus_fault(uint32_t fault_code, const fault_context_t *context)
{
  bsp_safe_outputs();
  safety_request_fault(AXIS_COUNT, fault_code, FAULT_SEVERITY_BUS_SAFE, context);
}

/**
 * @brief 读取指定轴故障位图。
 *
 * @param axis 轴编号。
 * @return 故障位图；非法轴返回 0。
 *
 * 调用上下文：任务。
 * 失败行为：非法轴返回 0。
 */
uint32_t safety_get_fault_flags(axis_id_t axis)
{
  uint32_t primask;
  uint32_t result;
  uint32_t domain = safety_domain_index(axis);

  if (domain >= SAFETY_DOMAIN_COUNT) {
    return 0u;
  }

  primask = safety_critical_enter();
  result = g_fault_flags[domain];
  safety_critical_exit(primask);
  return result;
}

/**
 * @brief 读取总线故障位图。
 *
 * @return 总线故障位图。
 *
 * 调用上下文：任务。
 * 失败行为：无失败路径。
 */
uint32_t safety_get_bus_fault_flags(void)
{
  return safety_get_fault_flags(AXIS_COUNT);
}

/**
 * @brief 判断指定轴是否有锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示存在故障或输入非法。
 *
 * 调用上下文：状态机和诊断任务。
 * 失败行为：非法轴返回 true。
 */
bool safety_axis_has_fault(axis_id_t axis)
{
  if ((axis != AXIS_A) && (axis != AXIS_B)) {
    return true;
  }

  return safety_get_fault_flags(axis) != 0u;
}

/**
 * @brief 清除指定故障位。
 *
 * @param axis 轴编号。
 * @param fault_mask 待清除掩码。
 * @return 无返回值。
 *
 * 调用上下文：仅人工恢复流程。
 * 失败行为：非法输入不修改状态。
 */
void safety_clear_faults(axis_id_t axis, uint32_t fault_mask)
{
  uint32_t primask;
  uint32_t domain = safety_domain_index(axis);

  if ((domain >= SAFETY_DOMAIN_COUNT) || (fault_mask == 0u)) {
    return;
  }

  primask = safety_critical_enter();
  g_fault_flags[domain] &= ~fault_mask;
  safety_critical_exit(primask);
}

/**
 * @brief 从队列弹出最早故障。
 *
 * @param record 输出记录。
 * @return true 表示读取成功。
 *
 * 调用上下文：日志和诊断任务。
 * 失败行为：记录指针为空或队列为空时返回 false。
 */
bool safety_event_pop(safety_fault_record_t *record)
{
  uint32_t primask;
  bool available;

  if (record == 0) {
    return false;
  }

  primask = safety_critical_enter();
  if ((g_queue_head == g_queue_tail) ||
      (g_queue_head >= SAFETY_FAULT_QUEUE_CAPACITY) ||
      (g_queue_tail >= SAFETY_FAULT_QUEUE_CAPACITY)) {
    available = false;
  } else {
    *record = g_fault_queue[g_queue_head];
    g_queue_head = (g_queue_head + 1u) % SAFETY_FAULT_QUEUE_CAPACITY;
    available = true;
  }
  safety_critical_exit(primask);

  return available;
}

/**
 * @brief 读取事件丢弃计数。
 *
 * @return 丢弃事件数。
 *
 * 调用上下文：诊断任务。
 * 失败行为：无失败路径。
 */
uint32_t safety_get_dropped_event_count(void)
{
  uint32_t primask;
  uint32_t result;

  primask = safety_critical_enter();
  result = g_dropped_events;
  safety_critical_exit(primask);
  return result;
}
