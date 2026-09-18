/**
 * @file app_scheduler.c
 * @brief 阶段 0 1kHz 服务实现。
 *
 * 主要接口：app_scheduler_step、app_scheduler_run。
 * 依赖关系：依赖 app_state_machine、test_self_check、test_a4_no_power、BSP 和 watchdog。
 * 关键安全约束：自检或无功率验证失败进入故障；看门狗只在完整服务后重载；A4 验证不使能功率。
 */

#include "app/app_scheduler.h"

#include "app/app_state_machine.h"
#include "bsp/bsp_safe_outputs.h"
#include "bsp/bsp_time.h"
#include "bsp/bsp_watchdog.h"
#include "test/test_a4_no_power.h"
#include "test/test_self_check.h"

/** @brief A4 无功率验证报告，静态分配。 */
static a4_no_power_report_t s_a4_report;

/** @brief A4 无功率验证只执行一次，避免重复采集零偏。 */
static bool s_a4_verified;

/**
 * @brief 执行一次 1kHz 步骤。
 *
 * @param timestamp_ms 当前毫秒时间。
 * @return 无返回值。
 *
 * 调用上下文：主循环。
 * 失败行为：自检失败提交 FAULT 事件。
 */
void app_scheduler_step(uint32_t timestamp_ms)
{
  axis_id_t axis;

  /* A4 无功率验证：上电后只执行一次，不使能任何功率输出。 */
  if (!s_a4_verified) {
    s_a4_verified = true;
    if (!test_a4_no_power_run(A4_NO_POWER_MASK_AUTORUN, &s_a4_report)) {
      for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
        (void)app_state_machine_post(axis, APP_EVENT_FAULT);
      }
    }
    test_a4_no_power_log_report(&s_a4_report);
  }

  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    app_state_t state = app_state_machine_get(axis);

    if ((state == APP_STATE_SELF_TEST) && !test_self_check_run(axis)) {
      (void)app_state_machine_post(axis, APP_EVENT_FAULT);
    } else if (state == APP_STATE_SELF_TEST) {
      (void)app_state_machine_post(axis, APP_EVENT_SELF_TEST_PASSED);
    }

    app_state_machine_service(axis, timestamp_ms);
  }

  /* 1Hz 心跳只用于阶段 0 状态可见性，不影响控制路径。 */
  bsp_status_led_set(((timestamp_ms / UINT32_C(500)) & UINT32_C(1)) != 0u);
  bsp_watchdog_reload();
}

/**
 * @brief 运行裸机调度循环。
 *
 * @return 无返回值。
 *
 * 调用上下文：main。
 * 失败行为：不会返回；上位机或调试器通过 SWD 诊断。
 */
void app_scheduler_run(void)
{
  uint32_t last_service_ms = bsp_time_get_ms();

  while (1) {
    uint32_t now_ms = bsp_time_get_ms();

    if ((uint32_t)(now_ms - last_service_ms) >= 1u) {
      last_service_ms = now_ms;
      app_scheduler_step(now_ms);
    }
  }
}
