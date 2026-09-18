/**
 * @file safety_speed.h
 * @brief 机械转速限速与超速故障判定接口。
 *
 * 主要接口：safety_speed_check。
 * 依赖关系：仅依赖 C11 浮点与整数类型，可在主机测试。
 * 关键安全约束：达到最高运行转速（默认 7000rpm）进入限速停机；
 *              超过最高故障转速（默认 7500rpm）锁存超速故障。
 */

#ifndef FOC_SAFETY_SPEED_H
#define FOC_SAFETY_SPEED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 转速判定结果。
 *
 * 取值范围：NORMAL 允许继续运行；LIMIT 触发限速停机；FAULT 触发超速故障锁存。
 */
typedef enum {
  SAFETY_SPEED_NORMAL = 0, /**< 转速低于限速点。 */
  SAFETY_SPEED_LIMIT,      /**< 达到或超过限速点，要求零转矩停机。 */
  SAFETY_SPEED_FAULT       /**< 超过超速故障点，要求锁存故障。 */
} safety_speed_verdict_t;

/**
 * @brief 按限速点和超速故障点判定当前转速。
 *
 * @param speed_rpm 当前机械转速，单位 rpm。
 * @param max_speed_rpm 最高运行转速，单位 rpm，必须大于 0。
 * @param over_speed_fault_rpm 超速故障转速，单位 rpm，必须大于 max_speed_rpm。
 * @param verdict 输出判定结果；允许为 NULL。
 * @return true 表示输入有效且判定成功。
 *
 * 调用上下文：20kHz 控制中断。
 * 失败行为：任一输入非有限、max 不大于 0 或 fault 不大于 max 时返回 false，
 *           输出不修改；调用方应按 FAULT 处理保持保守。
 */
bool safety_speed_check(float speed_rpm,
                        float max_speed_rpm,
                        float over_speed_fault_rpm,
                        safety_speed_verdict_t *verdict);

#ifdef __cplusplus
}
#endif

#endif
