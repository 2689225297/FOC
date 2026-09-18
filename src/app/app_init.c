/**
 * @file app_init.c
 * @brief 产品固件启动顺序和参数加载实现。
 *
 * 主要接口：app_init。
 * 依赖关系：依赖 BSP、安全、状态机、存储和自检模块。
 * 关键安全约束：PB12 最早拉低；PWM/ADC 与电流采样仅在无功率条件下初始化；
 *              参数 CRC 或标定位失败时不得进入 IDLE 或闭环。
 */

#include "app/app_init.h"

#include "app/app_control_core.h"
#include "app/app_profiling.h"
#include "app/app_state_machine.h"
#include "bsp/bsp_clock.h"
#include "bsp/bsp_current_sense.h"
#include "bsp/bsp_drv8323.h"
#include "bsp/bsp_flash.h"
#include "bsp/bsp_log.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_safe_outputs.h"
#include "bsp/bsp_time.h"
#include "bsp/bsp_watchdog.h"
#include "comm/comm_guard.h"
#include "motor/motor_axis.h"
#include "safety/safety.h"
#include "storage/storage_parameter.h"
#include "storage/storage_transaction.h"
#include "test/test_profiling.h"

/** @brief 固件版本编码，与状态机故障上下文保持一致。 */
#define APP_INIT_FIRMWARE_VERSION UINT16_C(0x0100)

/** @brief 启动参数加载使用的暂存记录长度。 */
#define APP_PARAMETER_RECORD_BUFFER_SIZE STORAGE_PARAMETER_RECORD_SIZE

/** @brief 20kHz 控制周期额定 DWT 周期数：192MHz / 20kHz。 */
#define APP_INIT_PROFILING_NOMINAL_CYCLES UINT32_C(9600)

/**
 * @brief 构造只包含时间和状态的故障上下文。
 *
 * @param timestamp_ms 当前毫秒时间。
 * @param state 当前状态。
 * @return 填充后的上下文。
 *
 * 调用上下文：系统初始化失败路径。
 * 失败行为：无失败路径。
 */
static fault_context_t app_init_make_context(uint32_t timestamp_ms, app_state_t state)
{
  fault_context_t context;

  context.timestamp_ms = timestamp_ms;
  context.speed_rpm = 0.0f;
  context.current_a = 0.0f;
  context.controller_state = state;
  context.firmware_version = APP_INIT_FIRMWARE_VERSION;
  context.reserved = 0u;
  return context;
}

/**
 * @brief 从两个参数槽选择序号更新的有效记录。
 *
 * @param selected 输出选中参数。
 * @param selected_sequence 输出选中序号。
 * @param default_parameters 输出未标定默认参数。
 * @return true 表示至少存在一个格式有效的记录。
 *
 * 调用上下文：启动阶段。
 * 失败行为：两个槽均无效时返回 false，并输出安全默认参数。
 */
static bool app_init_load_parameter_record(parameter_set_t *selected,
                                           uint32_t *selected_sequence,
                                           parameter_set_t *default_parameters)
{
  uint8_t record[APP_PARAMETER_RECORD_BUFFER_SIZE];
  parameter_set_t candidate;
  parameter_set_t defaults;
  uint32_t sequence;
  bool slot_a_valid = false;
  bool slot_b_valid = false;
  bool selected_val = false;
  uint32_t slot_a_sequence = 0u;
  uint32_t slot_b_sequence = 0u;

  if ((selected == 0) || (selected_sequence == 0) || (default_parameters == 0)) {
    return false;
  }

  storage_parameter_defaults(&defaults);

  if (bsp_flash_read(STORAGE_PARAMETER_SLOT_A_ADDRESS, record, sizeof(record)) &&
      storage_parameter_parse_record(record, sizeof(record), &candidate, &sequence)) {
    *selected = candidate;
    slot_a_sequence = sequence;
    slot_a_valid = true;
  }

  if (bsp_flash_read(STORAGE_PARAMETER_SLOT_B_ADDRESS, record, sizeof(record)) &&
      storage_parameter_parse_record(record, sizeof(record), &candidate, &sequence)) {
    if (!slot_a_valid || ((int32_t)(sequence - slot_a_sequence) > 0)) {
      *selected = candidate;
      slot_b_sequence = sequence;
      slot_b_valid = true;
    }
  }

  if (slot_b_valid) {
    *selected_sequence = slot_b_sequence;
    selected_val = true;
  } else if (slot_a_valid) {
    *selected_sequence = slot_a_sequence;
    selected_val = true;
  } else {
    *selected = defaults;
    *selected_sequence = 0u;
  }

  *default_parameters = defaults;
  return selected_val;
}

/**
 * @brief 按安全顺序初始化产品固件。
 *
 * @return true 表示允许进入校准流程。
 *
 * 调用上下文：main。
 * 失败行为：任何失败都锁存故障并保持安全输出。
 */
bool app_init(void)
{
  parameter_set_t parameters;
  parameter_set_t defaults;
  uint32_t sequence = 0u;
  uint32_t timestamp_ms;
  fault_context_t context;
  bool initialized = true;
  axis_id_t axis;

  /* 第一条产品动作必须先隔离 B 路并从 A 路输入撤掉有效 PWM。 */
  bsp_safe_outputs_early();
  safety_init();

  if (!bsp_clock_init()) {
    context = app_init_make_context(0u, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_INTERNAL, FAULT_SEVERITY_BUS_SAFE, &context);
    bsp_status_led_set(true);
    return false;
  }

  bsp_gpio_init();
  if (!bsp_log_init()) {
    context = app_init_make_context(0u, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_INTERNAL, FAULT_SEVERITY_BUS_SAFE, &context);
    return false;
  }

  if (!bsp_time_init()) {
    context = app_init_make_context(0u, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_INTERNAL, FAULT_SEVERITY_BUS_SAFE, &context);
    return false;
  }

  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    if (!app_state_machine_init(axis) || !motor_axis_init(axis)) {
      initialized = false;
    }
  }

  if (!initialized || !bsp_watchdog_init() || !bsp_drv8323_init_safe()) {
    context = app_init_make_context(bsp_time_get_ms(), APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_INTERNAL, FAULT_SEVERITY_BUS_SAFE, &context);
    initialized = false;
  }

  /* A4 无功率 BSP：装订 PWM/ADC 时基与电流采样链路，结束时输出仍被安全底座禁用。 */
  if (!bsp_pwm_init() || !bsp_current_sense_init()) {
    context = app_init_make_context(bsp_time_get_ms(), APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_INTERNAL, FAULT_SEVERITY_HARDWARE, &context);
    initialized = false;
  }

  /* A6 双轴控制核心、通信隔离、Profiling 与参数事务初始化（控制中断使能前完成）。 */
  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    if (!app_control_core_init(axis)) {
      initialized = false;
    }
  }
  comm_guard_init();
  (void)test_profiling_init();
  app_profiling_init(APP_INIT_PROFILING_NOMINAL_CYCLES);
  {
    storage_transaction_io_t tx_io;

    tx_io.read = bsp_flash_read;
    tx_io.erase = 0;
    tx_io.program = 0;
    storage_transaction_init(&tx_io);
  }

  timestamp_ms = bsp_time_get_ms();
  if (bsp_clock_take_watchdog_reset_flag()) {
    context = app_init_make_context(timestamp_ms, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_WATCHDOG_RESET, FAULT_SEVERITY_BUS_SAFE, &context);
    initialized = false;
  }

  if (!app_init_load_parameter_record(&parameters, &sequence, &defaults)) {
    for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
      (void)motor_axis_set_parameters(axis, &defaults);
    }
    context = app_init_make_context(timestamp_ms, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_PARAMETER_CRC, FAULT_SEVERITY_HARDWARE, &context);
    initialized = false;
  } else if (!storage_parameter_is_operational(&parameters)) {
    for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
      (void)motor_axis_set_parameters(axis, &parameters);
    }
    context = app_init_make_context(timestamp_ms, APP_STATE_POWER_ON);
    safety_request_fault(AXIS_COUNT, FAULT_CALIBRATION_INVALID, FAULT_SEVERITY_HARDWARE, &context);
    initialized = false;
  } else {
    for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
      (void)motor_axis_set_parameters(axis, &parameters);
    }
    (void)sequence;
  }

  (void)bsp_log_write_nonblocking("FOC safe boot\r\n", 15u);

  (void)app_state_machine_post(AXIS_A, APP_EVENT_POWER_ON_COMPLETE);
  (void)app_state_machine_post(AXIS_B, APP_EVENT_POWER_ON_COMPLETE);

  if (!initialized || (safety_get_bus_fault_flags() != 0u)) {
    (void)app_state_machine_post(AXIS_A, APP_EVENT_FAULT);
    (void)app_state_machine_post(AXIS_B, APP_EVENT_FAULT);
  }

  return initialized;
}
