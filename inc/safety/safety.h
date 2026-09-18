/**
 * @file safety.h
 * @brief 故障锁存、故障事件和快速安全动作接口。
 *
 * 主要接口：故障请求、故障位查询、故障清除和事件出队。
 * 依赖关系：依赖 motor_types.h；快速故障函数还会调用 BSP 安全输出接口。
 * 关键安全约束：故障请求必须可在中断上下文调用，且不得分配内存、等待锁或执行阻塞 I/O。
 */

#ifndef FOC_SAFETY_H
#define FOC_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 故障事件队列容量，单位个。 */
#define SAFETY_FAULT_QUEUE_CAPACITY ((uint32_t)32u)

/**
 * @brief 可持久化或通过协议发送的完整故障记录。
 */
typedef struct {
  axis_id_t axis;             /**< 故障所属轴；总线故障使用 AXIS_COUNT 表示。 */
  uint32_t fault_code;        /**< fault_code_t 单一位或兼容组合。 */
  fault_severity_t severity;  /**< 故障严重度。 */
  fault_context_t context;    /**< 发生时的转速、电流、状态和时间。 */
  uint16_t sequence;          /**< 故障事件序号，用于诊断队列排序。 */
  uint16_t reserved;          /**< 预留字段，写入 0。 */
} safety_fault_record_t;

/**
 * @brief 初始化故障管理器的静态状态。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统初始化、在中断使能前调用。
 * 失败行为：无失败路径；所有队列和锁存位被清零。
 */
void safety_init(void);

/**
 * @brief 请求并锁存一个故障。
 *
 * @param axis 故障所属轴；总线故障传 AXIS_COUNT。
 * @param fault_code 故障位，必须来自 fault_code_t。
 * @param severity 故障严重度。
 * @param context 故障上下文；允许为 NULL，此时使用全零上下文。
 * @return 无返回值。
 *
 * 调用上下文：允许从中断和任务调用。
 * 失败行为：队列满时保留原始锁存位并增加丢弃计数，不阻塞调用者。
 */
void safety_request_fault(axis_id_t axis,
                          uint32_t fault_code,
                          fault_severity_t severity,
                          const fault_context_t *context);

/**
 * @brief nFAULT 或其他最低延迟故障路径的公共入口。
 *
 * @param fault_code 故障位。
 * @param context 故障上下文；允许为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：最高优先级中断。
 * 失败行为：先执行全轴安全输出，再锁存故障；任何队列操作都不得阻塞安全动作。
 */
void safety_fast_fault(uint32_t fault_code, const fault_context_t *context);

/**
 * @brief 总线级或 CPU 异常的快速故障入口。
 *
 * @param fault_code 故障位。
 * @param context 故障上下文；允许为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：HardFault、NMI 或其他总线级异常。
 * 失败行为：先执行全轴安全输出，再锁存总线故障；函数不得访问堆栈以外的阻塞资源。
 */
void safety_fast_bus_fault(uint32_t fault_code, const fault_context_t *context);

/**
 * @brief 读取指定轴的锁存故障位图。
 *
 * @param axis 轴编号。
 * @return 该轴故障位图；非法轴返回 0。
 *
 * 调用上下文：任务和诊断路径。
 * 失败行为：非法轴返回 0，不修改故障状态。
 */
uint32_t safety_get_fault_flags(axis_id_t axis);

/**
 * @brief 读取总线级锁存故障位图。
 *
 * @return 总线故障位图。
 *
 * 调用上下文：任务和诊断路径。
 * 失败行为：无失败路径。
 */
uint32_t safety_get_bus_fault_flags(void);

/**
 * @brief 判断指定轴是否存在任何锁存故障。
 *
 * @param axis 轴编号。
 * @return true 表示存在锁存故障。
 *
 * 调用上下文：状态机和通信诊断。
 * 失败行为：非法轴返回 true，以保持保守安全策略。
 */
bool safety_axis_has_fault(axis_id_t axis);

/**
 * @brief 在人工恢复流程中清除指定故障位。
 *
 * @param axis 轴编号。
 * @param fault_mask 允许清除的位图。
 * @return 无返回值。
 *
 * 调用上下文：仅允许 RECOVERY 状态的任务调用。
 * 失败行为：非法轴或掩码不修改状态；不可清除位仍保持锁存。
 */
void safety_clear_faults(axis_id_t axis, uint32_t fault_mask);

/**
 * @brief 从故障事件队列读取最早记录。
 *
 * @param record 输出记录。
 * @return true 表示读取成功；false 表示队列为空或参数非法。
 *
 * 调用上下文：日志和诊断任务。
 * 失败行为：空队列返回 false，不等待。
 */
bool safety_event_pop(safety_fault_record_t *record);

/**
 * @brief 读取因队列满而丢弃的事件数量。
 *
 * @return 丢弃计数，单位个。
 *
 * 调用上下文：诊断和性能统计。
 * 失败行为：无失败路径。
 */
uint32_t safety_get_dropped_event_count(void);

#ifdef __cplusplus
}
#endif

#endif
