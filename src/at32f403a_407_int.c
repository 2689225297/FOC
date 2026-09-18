/**
 * @file at32f403a_407_int.c
 * @brief AT32F403A 异常和产品中断入口。
 *
 * 主要接口：SysTick、TMR1 更新（双轴 20kHz 控制调度）、HardFault 类异常和 PA15 nFAULT 中断。
 * 依赖关系：依赖 BSP、安全、状态机、控制核心、通信隔离与 Profiling 模块。
 * 关键安全约束：nFAULT 中断先执行安全输出和锁存，再提交状态机事件；
 *               20kHz 控制中断内禁止调用 bsp_log 等非阻塞约束外接口。
 */

#include "at32f403a_407.h"

#include <string.h>

#include "app/app_control_core.h"
#include "app/app_profiling.h"
#include "app/app_state_machine.h"
#include "bsp/bsp_current_sense.h"
#include "bsp/bsp_time.h"
#include "comm/comm_guard.h"
#include "motor/motor_axis.h"
#include "motor/motor_control_path.h"
#include "motor/motor_current.h"
#include "motor/motor_math.h"
#include "safety/safety.h"
#include "test/test_profiling.h"

/** @brief 中断故障上下文使用的固件版本。 */
#define AT32_INT_FIRMWARE_VERSION UINT16_C(0x0100)

/** @brief 双轴 20kHz 控制周期，单位 s。 */
#define A6_CONTROL_DT_S (0.00005f)

/** @brief 20kHz 控制周期额定 DWT 周期数：192MHz / 20kHz。 */
#define A6_PROFILING_NOMINAL_CYCLES UINT32_C(9600)

/** @brief CAN 命令超时阈值，单位 ms。 */
#define A6_CAN_TIMEOUT_MS UINT32_C(100)

/**
 * @brief 构造中断可用的最小故障上下文。
 *
 * @param timestamp_ms 当前毫秒时间。
 * @return 全零转速和电流的上下文。
 *
 * 调用上下文：异常和 nFAULT 中断。
 * 失败行为：无失败路径。
 */
static fault_context_t at32_int_context(uint32_t timestamp_ms)
{
  fault_context_t context;

  context.timestamp_ms = timestamp_ms;
  context.speed_rpm = 0.0f;
  context.current_a = 0.0f;
  context.controller_state = APP_STATE_FAULT;
  context.firmware_version = AT32_INT_FIRMWARE_VERSION;
  context.reserved = 0u;
  return context;
}

/**
 * @brief 不可屏蔽中断入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：NMI 异常。
 * 失败行为：关闭双轴输出、锁存内部故障并停在原地等待复位。
 */
void NMI_Handler(void)
{
  fault_context_t context = at32_int_context(bsp_time_get_ms());

  safety_fast_bus_fault(FAULT_INTERNAL, &context);
  while (1) {
  }
}

/**
 * @brief HardFault 入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：HardFault 异常。
 * 失败行为：关闭双轴输出并停在原地，不复位或自动重试。
 */
void HardFault_Handler(void)
{
  fault_context_t context = at32_int_context(bsp_time_get_ms());

  safety_fast_bus_fault(FAULT_INTERNAL, &context);
  while (1) {
  }
}

/**
 * @brief 存储管理异常入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：MemManage 异常。
 * 失败行为：关闭输出并停在原地等待诊断。
 */
void MemManage_Handler(void)
{
  fault_context_t context = at32_int_context(bsp_time_get_ms());

  safety_fast_bus_fault(FAULT_INTERNAL, &context);
  while (1) {
  }
}

/**
 * @brief 总线异常入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：BusFault 异常。
 * 失败行为：关闭输出并停在原地等待诊断。
 */
void BusFault_Handler(void)
{
  fault_context_t context = at32_int_context(bsp_time_get_ms());

  safety_fast_bus_fault(FAULT_INTERNAL, &context);
  while (1) {
  }
}

/**
 * @brief 用法异常入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：UsageFault 异常。
 * 失败行为：关闭输出并停在原地等待诊断。
 */
void UsageFault_Handler(void)
{
  fault_context_t context = at32_int_context(bsp_time_get_ms());

  safety_fast_bus_fault(FAULT_INTERNAL, &context);
  while (1) {
  }
}

/**
 * @brief SVC 异常占位入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：FreeRTOS 接入后由内核接管。
 * 失败行为：当前阶段直接返回。
 */
void SVC_Handler(void)
{
}

/**
 * @brief 调试监视异常占位入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：调试器使用。
 * 失败行为：当前阶段直接返回。
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief PendSV 异常占位入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：FreeRTOS 接入后由内核接管。
 * 失败行为：当前阶段直接返回。
 */
void PendSV_Handler(void)
{
}

/**
 * @brief SysTick 中断入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：1kHz 系统节拍。
 * 失败行为：无失败路径；只递增时基。
 */
void SysTick_Handler(void)
{
  bsp_time_tick_isr();
}

/**
 * @brief PA15 nFAULT 下降沿中断入口。
 *
 * @return 无返回值。
 *
 * 调用上下文：DRV8323 故障。
 * 失败行为：先关闭 B 路并锁存硬件故障，再向 B 轴状态机提交 FAULT 事件。
 */
void EXINT15_10_IRQHandler(void)
{
  if (exint_flag_get(EXINT_LINE_15) == SET) {
    fault_context_t context = at32_int_context(bsp_time_get_ms());

    exint_flag_clear(EXINT_LINE_15);
    safety_fast_fault(FAULT_DRV8323_NFAULT, &context);
    (void)app_state_machine_post(AXIS_B, APP_EVENT_FAULT);
  }
}

/**
 * @brief TMR1 更新中断：双轴 20kHz 控制调度入口。
 *
 * 调度顺序（每轴）：电流采样换算 → app_control_core_step（观测器仲裁与电流环）
 * → app_control_core_speed_govern（限速/超速）→ CAN 超时检查与转矩归零；
 * 末尾记录 Profiling 周期与忙时。
 * 关键安全约束：本中断内禁止调用 bsp_log 等非阻塞约束外接口；
 *               任一轴故障锁存只影响本轴，不得阻塞或破坏另一轴。
 */
void TMR1_OVF_TMR10_IRQHandler(void)
{
  static uint32_t s_last_cycles;
  uint32_t isr_start_cycles;
  uint32_t period_cycles;
  uint32_t busy_cycles;
  axis_id_t axis;

  if (tmr_flag_get(TMR1, TMR_OVF_FLAG) == RESET) {
    return;
  }
  tmr_flag_clear(TMR1, TMR_OVF_FLAG);

  isr_start_cycles = test_profiling_read_cycles();
  period_cycles = test_profiling_elapsed_cycles(s_last_cycles, isr_start_cycles);
  s_last_cycles = isr_start_cycles;

  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    motor_abc_t i_abc;
    uint16_t raw_u = 0u;
    uint16_t raw_v = 0u;
    motor_control_path_status_t path_status;
    parameter_set_t parameters;
    float speed_rpm;

    if (bsp_current_sense_read_raw(axis, 0u, &raw_u) &&
        bsp_current_sense_read_raw(axis, 1u, &raw_v) &&
        motor_axis_get_parameters(axis, &parameters)) {
      float voltage_u = ((float)raw_u) * MOTOR_CURRENT_ADC_VREF_V /
                        (float)MOTOR_CURRENT_ADC_FULL_SCALE;
      float voltage_v = ((float)raw_v) * MOTOR_CURRENT_ADC_VREF_V /
                        (float)MOTOR_CURRENT_ADC_FULL_SCALE;
      float iu_a = (voltage_u - parameters.current_calibration[axis].current_zero_offset_v) *
                   parameters.current_calibration[axis].current_gain_a_per_v *
                   parameters.current_calibration[axis].current_polarity;
      float iv_a = (voltage_v - parameters.current_calibration[axis].current_zero_offset_v) *
                   parameters.current_calibration[axis].current_gain_a_per_v *
                   parameters.current_calibration[axis].current_polarity;

      i_abc.a = iu_a;
      i_abc.b = iv_a;
      i_abc.c = -(iu_a + iv_a);
    } else {
      memset(&i_abc, 0, sizeof(i_abc));
    }

    (void)app_control_core_step(axis, &i_abc, A6_CONTROL_DT_S);

    if (motor_control_path_get_status(axis, &path_status)) {
      speed_rpm = path_status.observer_speed_rpm;
    } else {
      speed_rpm = 0.0f;
    }
    app_control_core_speed_govern(axis, speed_rpm);

    if (comm_guard_can_timeout_check(bsp_time_get_ms(), A6_CAN_TIMEOUT_MS)) {
      app_control_core_can_timeout_zero(axis);
    }
  }

  busy_cycles = test_profiling_elapsed_cycles(isr_start_cycles, test_profiling_read_cycles());
  app_profiling_tick(period_cycles, busy_cycles);
}
