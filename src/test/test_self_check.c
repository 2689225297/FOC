/**
 * @file test_self_check.c
 * @brief 阶段 0 上电自检实现。
 *
 * 主要接口：test_self_check_run。
 * 依赖关系：依赖 BSP、存储参数和电机类型。
 * 关键安全约束：自检只验证无功率条件，不替代 A4 的示波器和目标板证据。
 */

#include "test/test_self_check.h"

#include <assert.h>
#include <stddef.h>

#include "bsp/bsp_drv8323.h"
#include "storage/storage_parameter.h"

/** @brief 编译期确认轴编号与遥测接口契约一致。 */
_Static_assert(AXIS_A == 0, "AXIS_A 编号必须冻结为 0");

/** @brief 编译期确认 B 轴编号。 */
_Static_assert(AXIS_B == 1, "AXIS_B 编号必须冻结为 1");

/** @brief 编译期确认轴总数。 */
_Static_assert(AXIS_COUNT == 2, "当前产品必须且只能有两个轴");

/** @brief 编译期确认单精度浮点格式占 4 字节。 */
_Static_assert(sizeof(float) == 4u, "参数记录要求 float 为 32 位");

/**
 * @brief 执行无功率上电自检。
 *
 * @param axis 轴编号。
 * @return true 表示自检通过。
 *
 * 调用上下文：SELF_TEST 状态的 1kHz 服务。
 * 失败行为：任何失败均返回 false，调用方保持 PWM 关闭。
 */
bool test_self_check_run(axis_id_t axis)
{
  parameter_set_t defaults;

  if ((axis != AXIS_A) && (axis != AXIS_B)) {
    return false;
  }

  if (sizeof(uint32_t) != 4u) {
    return false;
  }

  if (!bsp_drv8323_nfault_is_clear()) {
    return false;
  }

  storage_parameter_defaults(&defaults);
  if (storage_parameter_is_operational(&defaults)) {
    return false;
  }

  /* 自检阶段允许校准尚未完成；DRV8323 的功率使能由 CALIBRATION 状态继续把关。 */
  return true;
}
