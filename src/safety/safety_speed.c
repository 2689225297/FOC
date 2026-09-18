/**
 * @file safety_speed.c
 * @brief 机械转速限速与超速故障判定实现。
 *
 * 阈值来自 parameter_set_t 的 max_speed_rpm 与 over_speed_fault_rpm，
 * 本模块只做纯判定，不直接访问参数或执行停机。
 */

#include "safety/safety_speed.h"

#include <math.h>

/**
 * @brief 判定浮点值是否有限。
 *
 * @param value 浮点值。
 * @return true 表示有限。
 */
static bool safety_speed_is_finite(float value)
{
#ifdef _MSC_VER
  return isfinite(value) != 0;
#else
  return isfinite(value);
#endif
}

/**
 * @brief 按限速点和超速故障点判定当前转速。
 *
 * @param speed_rpm 当前机械转速，单位 rpm。
 * @param max_speed_rpm 最高运行转速，单位 rpm。
 * @param over_speed_fault_rpm 超速故障转速，单位 rpm。
 * @param verdict 输出判定结果；允许为 NULL。
 * @return true 表示输入有效且判定成功。
 */
bool safety_speed_check(float speed_rpm,
                        float max_speed_rpm,
                        float over_speed_fault_rpm,
                        safety_speed_verdict_t *verdict)
{
  if (!safety_speed_is_finite(speed_rpm) ||
      !safety_speed_is_finite(max_speed_rpm) ||
      !safety_speed_is_finite(over_speed_fault_rpm) ||
      (max_speed_rpm <= 0.0f) ||
      (over_speed_fault_rpm <= max_speed_rpm)) {
    return false;
  }

  if (speed_rpm >= over_speed_fault_rpm) {
    if (verdict != 0) {
      *verdict = SAFETY_SPEED_FAULT;
    }
  } else if (speed_rpm >= max_speed_rpm) {
    if (verdict != 0) {
      *verdict = SAFETY_SPEED_LIMIT;
    }
  } else if (verdict != 0) {
    *verdict = SAFETY_SPEED_NORMAL;
  }

  return true;
}
