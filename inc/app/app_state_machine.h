/**
 * @file app_state_machine.h
 * @brief 单轴运行状态机事件和查询接口。
 *
 * 主要接口：初始化、提交事件、1kHz 服务和读取当前状态。
 * 依赖关系：依赖 motor_types.h 和 safety.h。
 * 关键安全约束：只有 app_state_machine.c 可以写状态；其他模块只能提交事件。
 */

#ifndef FOC_APP_STATE_MACHINE_H
#define FOC_APP_STATE_MACHINE_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 单轴事件队列容量，单位个。 */
#define APP_STATE_EVENT_QUEUE_CAPACITY ((uint32_t)8u)

/**
 * @brief 状态机事件。
 *
 * 事件只表达意图，最终状态由冻结转移表决定。
 */
typedef enum {
  APP_EVENT_POWER_ON_COMPLETE = 0,     /**< 时钟和安全 GPIO 初始化完成。 */
  APP_EVENT_SELF_TEST_PASSED,          /**< 自检通过。 */
  APP_EVENT_CALIBRATION_PASSED,        /**< ADC 和驱动器校准通过。 */
  APP_EVENT_START_REQUEST_IF,          /**< 请求 I/F 启动。 */
  APP_EVENT_START_REQUEST_HFI,         /**< 请求 HFI 启动。 */
  APP_EVENT_ALIGN_TO_IF,               /**< 预定位完成并进入 I/F。 */
  APP_EVENT_ALIGN_TO_HFI,              /**< 预定位完成并进入 HFI。 */
  APP_EVENT_HFI_CONFIDENCE_LOST,       /**< HFI 置信度不足，回退 I/F。 */
  APP_EVENT_OBSERVER_READY,            /**< 观测器置信度满足闭环要求。 */
  APP_EVENT_ENTER_FIELD_WEAKENING,     /**< 在限幅内进入弱磁。 */
  APP_EVENT_EXIT_FIELD_WEAKENING,      /**< 退出弱磁并恢复 Id 参考。 */
  APP_EVENT_BRAKE_REQUEST,             /**< 请求受控制动。 */
  APP_EVENT_BRAKE_COMPLETE,            /**< 制动结束。 */
  APP_EVENT_COAST_REQUEST,             /**< 请求自由停车。 */
  APP_EVENT_STOP_COMPLETE,             /**< 转子已安全停止或允许回到空闲。 */
  APP_EVENT_START_FAILED,              /**< 启动策略失败。 */
  APP_EVENT_FAULT,                     /**< 立即进入 FAULT。 */
  APP_EVENT_RECOVERY_REQUEST,          /**< 人工恢复请求。 */
  APP_EVENT_RECOVERY_REQUIRES_CAL,     /**< 恢复需要重新校准。 */
  APP_EVENT_RECOVERY_CONFIRMED,        /**< 故障清除且目标转矩为零。 */
  APP_EVENT_RECOVERY_FAILED,           /**< 恢复复检失败。 */
  APP_EVENT_COUNT                      /**< 事件数量哨兵。 */
} app_event_t;

/**
 * @brief 初始化指定轴状态机和事件队列。
 *
 * @param axis 轴编号。
 * @return true 表示初始化成功。
 *
 * 调用上下文：系统初始化、在中断使能前调用。
 * 失败行为：非法轴返回 false，不修改其他轴。
 */
bool app_state_machine_init(axis_id_t axis);

/**
 * @brief 提交状态机事件。
 *
 * @param axis 轴编号。
 * @param event 事件编号。
 * @return true 表示事件已入队。
 *
 * 调用上下文：允许从中断和任务调用。
 * 失败行为：非法参数或队列满时返回 false；故障事件由 safety 模块另行锁存。
 */
bool app_state_machine_post(axis_id_t axis, app_event_t event);

/**
 * @brief 在 1kHz 服务周期消费事件并执行状态转移。
 *
 * @param axis 轴编号。
 * @param timestamp_ms 当前单调毫秒时间。
 * @return 无返回值。
 *
 * 调用上下文：高优先级慢速任务；不得在 20kHz 中断调用。
 * 失败行为：非法转移锁存 FAULT_ILLEGAL_STATE 并进入 FAULT。
 */
void app_state_machine_service(axis_id_t axis, uint32_t timestamp_ms);

/**
 * @brief 读取指定轴当前状态。
 *
 * @param axis 轴编号。
 * @return 当前状态；非法轴返回 APP_STATE_FAULT。
 *
 * 调用上下文：中断、任务和遥测。
 * 失败行为：非法轴返回 FAULT，保持保守。
 */
app_state_t app_state_machine_get(axis_id_t axis);

/**
 * @brief 返回状态的稳定诊断名称。
 *
 * @param state 状态编号。
 * @return 只读字符串；非法状态返回 "INVALID"。
 *
 * 调用上下文：日志和非实时诊断。
 * 失败行为：非法状态不访问数组越界。
 */
const char *app_state_name(app_state_t state);

#ifdef __cplusplus
}
#endif

#endif
