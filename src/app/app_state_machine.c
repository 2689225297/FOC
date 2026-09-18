/**
 * @file app_state_machine.c
 * @brief 单轴状态机、冻结转移表和唯一状态写者实现。
 *
 * 主要接口：app_state_machine_init、app_state_machine_post、app_state_machine_service。
 * 依赖关系：依赖 app_state_machine.h、safety.h 和 BSP 安全输出接口。
 * 关键安全约束：任何非法转移立即锁存故障并关闭对应轴输出。
 */

#include "app/app_state_machine.h"

#include "bsp/bsp_safe_outputs.h"
#include "safety/safety.h"

#ifndef FOC_HOST_TEST
#include "at32f403a_407.h"
#endif

/** @brief 每轴当前状态，只有本文件写入。 */
static volatile app_state_t g_axis_state[MOTOR_AXIS_COUNT];

/** @brief 每轴事件环形队列。 */
static volatile app_event_t g_axis_events[MOTOR_AXIS_COUNT][APP_STATE_EVENT_QUEUE_CAPACITY];

/** @brief 每轴事件读索引。 */
static volatile uint32_t g_event_head[MOTOR_AXIS_COUNT];

/** @brief 每轴事件写索引。 */
static volatile uint32_t g_event_tail[MOTOR_AXIS_COUNT];

/** @brief 固件版本编码，用于故障上下文；高字节为主版本，低字节为次版本。 */
#define APP_FIRMWARE_VERSION UINT16_C(0x0100)

/**
 * @brief 进入短临界区。
 *
 * @return 进入前 PRIMASK；主机测试下返回 0。
 *
 * 调用上下文：中断和任务。
 * 失败行为：无失败路径。
 */
static uint32_t app_sm_critical_enter(void)
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
 * @param primask app_sm_critical_enter 返回值。
 * @return 无返回值。
 *
 * 调用上下文：与进入函数成对调用。
 * 失败行为：无失败路径。
 */
static void app_sm_critical_exit(uint32_t primask)
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
 * @param axis 输入轴编号。
 * @return true 表示 AXIS_A 或 AXIS_B。
 *
 * 调用上下文：状态机所有公开接口。
 * 失败行为：无失败路径。
 */
static bool app_sm_axis_valid(axis_id_t axis)
{
  return (axis == AXIS_A) || (axis == AXIS_B);
}

/**
 * @brief 判断状态是否必须关闭功率输出。
 *
 * @param state 输入状态。
 * @return true 表示进入该状态时必须保持 A 路 PWM 无效和 B 路 PB12 低。
 *
 * 调用上下文：状态进入动作。
 * 失败行为：非法状态按 true 处理。
 */
static bool app_sm_state_is_output_safe(app_state_t state)
{
  return (state == APP_STATE_POWER_ON) ||
         (state == APP_STATE_SELF_TEST) ||
         (state == APP_STATE_CALIBRATION) ||
         (state == APP_STATE_IDLE) ||
         (state == APP_STATE_COAST) ||
         (state == APP_STATE_FAULT);
}

/**
 * @brief 根据当前状态和事件计算下一状态。
 *
 * @param current 当前状态。
 * @param event 输入事件。
 * @param next_state 输出下一状态。
 * @return true 表示转移合法；false 表示非法转移。
 *
 * 调用上下文：app_state_machine_service。
 * 失败行为：非法输入返回 false，不写 next_state。
 */
static bool app_sm_transition(app_state_t current,
                              app_event_t event,
                              app_state_t *next_state)
{
  if (next_state == 0) {
    return false;
  }

  if (event == APP_EVENT_FAULT) {
    *next_state = APP_STATE_FAULT;
    return true;
  }

  switch (current) {
    case APP_STATE_POWER_ON:
      if (event == APP_EVENT_POWER_ON_COMPLETE) {
        *next_state = APP_STATE_SELF_TEST;
        return true;
      }
      break;

    case APP_STATE_SELF_TEST:
      if (event == APP_EVENT_SELF_TEST_PASSED) {
        *next_state = APP_STATE_CALIBRATION;
        return true;
      }
      break;

    case APP_STATE_CALIBRATION:
      if (event == APP_EVENT_CALIBRATION_PASSED) {
        *next_state = APP_STATE_IDLE;
        return true;
      }
      break;

    case APP_STATE_IDLE:
      if ((event == APP_EVENT_START_REQUEST_IF) ||
          (event == APP_EVENT_START_REQUEST_HFI)) {
        *next_state = APP_STATE_ALIGN;
        return true;
      }
      if (event == APP_EVENT_OBSERVER_READY) {
        *next_state = APP_STATE_CLOSED_LOOP;
        return true;
      }
      if (event == APP_EVENT_COAST_REQUEST) {
        *next_state = APP_STATE_COAST;
        return true;
      }
      break;

    case APP_STATE_ALIGN:
      if (event == APP_EVENT_ALIGN_TO_IF) {
        *next_state = APP_STATE_OPEN_LOOP_IF;
        return true;
      }
      if (event == APP_EVENT_ALIGN_TO_HFI) {
        *next_state = APP_STATE_HFI_CLOSED_LOOP;
        return true;
      }
      break;

    case APP_STATE_OPEN_LOOP_IF:
      if (event == APP_EVENT_OBSERVER_READY) {
        *next_state = APP_STATE_BLEND;
        return true;
      }
      break;

    case APP_STATE_HFI_CLOSED_LOOP:
      if (event == APP_EVENT_HFI_CONFIDENCE_LOST) {
        *next_state = APP_STATE_OPEN_LOOP_IF;
        return true;
      }
      if (event == APP_EVENT_OBSERVER_READY) {
        *next_state = APP_STATE_BLEND;
        return true;
      }
      break;

    case APP_STATE_BLEND:
      if (event == APP_EVENT_OBSERVER_READY) {
        *next_state = APP_STATE_CLOSED_LOOP;
        return true;
      }
      break;

    case APP_STATE_CLOSED_LOOP:
      if (event == APP_EVENT_ENTER_FIELD_WEAKENING) {
        *next_state = APP_STATE_FIELD_WEAKENING;
        return true;
      }
      if (event == APP_EVENT_BRAKE_REQUEST) {
        *next_state = APP_STATE_BRAKE;
        return true;
      }
      if (event == APP_EVENT_COAST_REQUEST) {
        *next_state = APP_STATE_COAST;
        return true;
      }
      break;

    case APP_STATE_FIELD_WEAKENING:
      if (event == APP_EVENT_EXIT_FIELD_WEAKENING) {
        *next_state = APP_STATE_CLOSED_LOOP;
        return true;
      }
      if (event == APP_EVENT_BRAKE_REQUEST) {
        *next_state = APP_STATE_BRAKE;
        return true;
      }
      if (event == APP_EVENT_COAST_REQUEST) {
        *next_state = APP_STATE_COAST;
        return true;
      }
      break;

    case APP_STATE_BRAKE:
      if (event == APP_EVENT_BRAKE_COMPLETE) {
        *next_state = APP_STATE_IDLE;
        return true;
      }
      if (event == APP_EVENT_COAST_REQUEST) {
        *next_state = APP_STATE_COAST;
        return true;
      }
      break;

    case APP_STATE_COAST:
      if (event == APP_EVENT_STOP_COMPLETE) {
        *next_state = APP_STATE_IDLE;
        return true;
      }
      break;

    case APP_STATE_FAULT:
      if (event == APP_EVENT_RECOVERY_REQUEST) {
        *next_state = APP_STATE_RECOVERY;
        return true;
      }
      break;

    case APP_STATE_RECOVERY:
      if (event == APP_EVENT_RECOVERY_REQUIRES_CAL) {
        *next_state = APP_STATE_CALIBRATION;
        return true;
      }
      if (event == APP_EVENT_RECOVERY_CONFIRMED) {
        *next_state = APP_STATE_IDLE;
        return true;
      }
      if (event == APP_EVENT_RECOVERY_FAILED) {
        *next_state = APP_STATE_FAULT;
        return true;
      }
      break;

    default:
      break;
  }

  if ((event == APP_EVENT_START_FAILED) &&
      (current != APP_STATE_FAULT) &&
      (current != APP_STATE_RECOVERY)) {
    *next_state = APP_STATE_FAULT;
    return true;
  }

  if ((event == APP_EVENT_COAST_REQUEST) &&
      (current != APP_STATE_POWER_ON) &&
      (current != APP_STATE_SELF_TEST) &&
      (current != APP_STATE_CALIBRATION) &&
      (current != APP_STATE_COAST) &&
      (current != APP_STATE_FAULT) &&
      (current != APP_STATE_RECOVERY)) {
    *next_state = APP_STATE_COAST;
    return true;
  }

  return false;
}

/**
 * @brief 执行状态进入副作用。
 *
 * @param axis 轴编号。
 * @param state 已经确认生效的状态。
 * @return 无返回值。
 *
 * 调用上下文：app_state_machine_service。
 * 失败行为：安全状态调用安全输出；带功率状态暂不在此处使能输出。
 */
static void app_sm_enter_state(axis_id_t axis, app_state_t state)
{
  (void)axis;

  if (app_sm_state_is_output_safe(state)) {
    bsp_safe_outputs();
  }
}

/**
 * @brief 初始化单轴状态机。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 *
 * 调用上下文：中断使能前。
 * 失败行为：非法轴返回 false。
 */
bool app_state_machine_init(axis_id_t axis)
{
  uint32_t index;

  if (!app_sm_axis_valid(axis)) {
    return false;
  }

  g_axis_state[axis] = APP_STATE_POWER_ON;
  g_event_head[axis] = 0u;
  g_event_tail[axis] = 0u;
  for (index = 0u; index < APP_STATE_EVENT_QUEUE_CAPACITY; ++index) {
    g_axis_events[axis][index] = APP_EVENT_COUNT;
  }

  return true;
}

/**
 * @brief 提交状态机事件。
 *
 * @param axis 轴编号。
 * @param event 事件编号。
 * @return true 表示事件已入队。
 *
 * 调用上下文：中断和任务。
 * 失败行为：非法或队列满返回 false。
 */
bool app_state_machine_post(axis_id_t axis, app_event_t event)
{
  uint32_t primask;
  uint32_t next_tail;
  bool accepted = false;

  if (!app_sm_axis_valid(axis) || (event >= APP_EVENT_COUNT)) {
    return false;
  }

  primask = app_sm_critical_enter();
  next_tail = (g_event_tail[axis] + 1u) % APP_STATE_EVENT_QUEUE_CAPACITY;
  if (next_tail != g_event_head[axis]) {
    g_axis_events[axis][g_event_tail[axis]] = event;
    g_event_tail[axis] = next_tail;
    accepted = true;
  }
  app_sm_critical_exit(primask);

  return accepted;
}

/**
 * @brief 在 1kHz 服务周期消费事件。
 *
 * @param axis 轴编号。
 * @param timestamp_ms 当前毫秒时间。
 * @return 无返回值。
 *
 * 调用上下文：慢速任务。
 * 失败行为：非法转移锁存 FAULT_ILLEGAL_STATE 并进入 FAULT。
 */
void app_state_machine_service(axis_id_t axis, uint32_t timestamp_ms)
{
  uint32_t primask;
  bool has_event;

  if (!app_sm_axis_valid(axis)) {
    return;
  }

  do {
    app_event_t event;
    app_state_t current;
    app_state_t next;

    primask = app_sm_critical_enter();
    has_event = g_event_head[axis] != g_event_tail[axis];
    if (has_event) {
      event = g_axis_events[axis][g_event_head[axis]];
      g_event_head[axis] = (g_event_head[axis] + 1u) % APP_STATE_EVENT_QUEUE_CAPACITY;
    } else {
      event = APP_EVENT_COUNT;
    }
    current = g_axis_state[axis];
    app_sm_critical_exit(primask);

    if (!has_event) {
      break;
    }

    if (!app_sm_transition(current, event, &next)) {
      fault_context_t context;
      context.timestamp_ms = timestamp_ms;
      context.speed_rpm = 0.0f;
      context.current_a = 0.0f;
      context.controller_state = current;
      context.firmware_version = APP_FIRMWARE_VERSION;
      context.reserved = 0u;
      safety_request_fault(axis, FAULT_ILLEGAL_STATE, FAULT_SEVERITY_AXIS_LATCHED, &context);
      next = APP_STATE_FAULT;
    }

    primask = app_sm_critical_enter();
    g_axis_state[axis] = next;
    app_sm_critical_exit(primask);
    app_sm_enter_state(axis, next);
  } while (has_event);
}

/**
 * @brief 读取轴当前状态。
 *
 * @param axis 轴编号。
 * @return 当前状态；非法轴返回 FAULT。
 *
 * 调用上下文：中断、任务和遥测。
 * 失败行为：非法轴返回保守状态。
 */
app_state_t app_state_machine_get(axis_id_t axis)
{
  uint32_t primask;
  app_state_t state;

  if (!app_sm_axis_valid(axis)) {
    return APP_STATE_FAULT;
  }

  primask = app_sm_critical_enter();
  state = g_axis_state[axis];
  app_sm_critical_exit(primask);
  return state;
}

/**
 * @brief 返回状态诊断名称。
 *
 * @param state 状态编号。
 * @return 静态只读字符串。
 *
 * 调用上下文：日志和诊断。
 * 失败行为：非法状态返回 "INVALID"。
 */
const char *app_state_name(app_state_t state)
{
  static const char *const names[APP_STATE_COUNT] = {
    "POWER_ON",
    "SELF_TEST",
    "CALIBRATION",
    "IDLE",
    "ALIGN",
    "OPEN_LOOP_IF",
    "HFI_CLOSED_LOOP",
    "BLEND",
    "CLOSED_LOOP",
    "FIELD_WEAKENING",
    "BRAKE",
    "COAST",
    "FAULT",
    "RECOVERY"
  };

  if ((state < APP_STATE_POWER_ON) || (state >= APP_STATE_COUNT)) {
    return "INVALID";
  }

  return names[state];
}
