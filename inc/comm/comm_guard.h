/**
 * @file comm_guard.h
 * @brief CAN、UART、USB、I2C 与日志通道隔离和 CAN 超时看门狗接口。
 *
 * 主要接口：通道健康位、发送/接收记账、CAN 喂狗与超时检查、日志节流。
 * 依赖关系：仅依赖 C11 固定宽度整数，不访问寄存器，可在主机测试。
 * 关键安全约束：任一通信通道故障只锁存该通道，不得阻塞或破坏其他通道与
 *              20kHz 控制中断；CAN 超时由控制中断周期性检查并触发转矩归零。
 */

#ifndef FOC_COMM_GUARD_H
#define FOC_COMM_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 通道数量。 */
#define COMM_CHANNEL_COUNT ((uint32_t)5u)

/**
 * @brief 通信与日志通道编号。
 *
 * 取值范围：CAN 到 LOG；LOG 也参与隔离，但节流策略独立。
 */
typedef enum {
  COMM_CHANNEL_CAN = 0,      /**< CAN 命令通道，故障后停止命令，控制侧进入转矩归零。 */
  COMM_CHANNEL_UART,         /**< UART 调试通道。 */
  COMM_CHANNEL_USB,          /**< USB 虚拟串口通道。 */
  COMM_CHANNEL_I2C,          /**< I2C 传感器或扩展通道。 */
  COMM_CHANNEL_LOG,          /**< 系统日志通道。 */
  COMM_CHANNEL_COUNT_SENTINEL /**< 通道数量哨兵，等于 COMM_CHANNEL_COUNT。 */
} comm_channel_t;

/**
 * @brief 初始化全部通道为启用且健康。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统初始化、中断使能前。
 * 失败行为：无失败路径；全部计数器清零。
 */
void comm_guard_init(void);

/**
 * @brief 启用或禁用通道。
 *
 * @param channel 通道编号。
 * @param enabled true 表示允许收发。
 * @return true 表示设置成功。
 *
 * 调用上下文：配置和诊断任务。
 * 失败行为：非法通道返回 false。
 */
bool comm_guard_set_channel_enabled(comm_channel_t channel, bool enabled);

/**
 * @brief 查询通道是否启用。
 *
 * @param channel 通道编号。
 * @return true 表示启用。
 *
 * 调用上下文：驱动和诊断任务。
 * 失败行为：非法通道返回 false。
 */
bool comm_guard_is_channel_enabled(comm_channel_t channel);

/**
 * @brief 锁存指定通道故障。
 *
 * @param channel 通道编号。
 * @return 无返回值。
 *
 * 调用上下文：通道驱动错误处理和诊断。
 * 失败行为：非法通道不修改任何状态；不影响其他通道。
 */
void comm_guard_channel_fault(comm_channel_t channel);

/**
 * @brief 清除指定通道锁存故障。
 *
 * @param channel 通道编号。
 * @return 无返回值。
 *
 * 调用上下文：恢复流程和诊断。
 * 失败行为：非法通道不修改任何状态。
 */
void comm_guard_channel_healthy(comm_channel_t channel);

/**
 * @brief 查询通道是否健康（启用且无锁存故障）。
 *
 * @param channel 通道编号。
 * @return true 表示健康。
 *
 * 调用上下文：控制中断和任务。
 * 失败行为：非法通道返回 false，保持保守。
 */
bool comm_guard_is_channel_healthy(comm_channel_t channel);

/**
 * @brief 记录一次发送并返回是否允许。
 *
 * @param channel 通道编号。
 * @param now_ms 当前单调毫秒时间。
 * @param byte_count 本次字节数。
 * @return true 表示允许发送。
 *
 * 调用上下文：各通道驱动发送前。
 * 失败行为：非法通道或未启用或已锁存时返回 false 并增加丢弃计数。
 */
bool comm_guard_tx_record(comm_channel_t channel, uint32_t now_ms, uint32_t byte_count);

/**
 * @brief 记录一次接收并返回是否允许处理。
 *
 * @param channel 通道编号。
 * @param now_ms 当前单调毫秒时间。
 * @param byte_count 本次字节数。
 * @return true 表示允许处理。
 *
 * 调用上下文：各通道驱动接收中断。
 * 失败行为：非法通道或未启用或已锁存时返回 false 并增加溢出计数。
 */
bool comm_guard_rx_record(comm_channel_t channel, uint32_t now_ms, uint32_t byte_count);

/**
 * @brief 读取通道发送丢弃计数。
 *
 * @param channel 通道编号。
 * @return 丢弃计数。
 *
 * 调用上下文：诊断。
 * 失败行为：非法通道返回 0。
 */
uint32_t comm_guard_tx_dropped(comm_channel_t channel);

/**
 * @brief 读取通道接收溢出计数。
 *
 * @param channel 通道编号。
 * @return 溢出计数。
 *
 * 调用上下文：诊断。
 * 失败行为：非法通道返回 0。
 */
uint32_t comm_guard_rx_overflow(comm_channel_t channel);

/**
 * @brief 记录 CAN 心跳帧，刷新超时基准。
 *
 * @param now_ms 当前单调毫秒时间。
 * @return 无返回值。
 *
 * 调用上下文：CAN 接收驱动或 1kHz 管理任务。
 * 失败行为：无失败路径；超时基准按单调时钟差计算。
 */
void comm_guard_can_feed(uint32_t now_ms);

/**
 * @brief 检查 CAN 是否超时。
 *
 * @param now_ms 当前单调毫秒时间。
 * @param timeout_ms 允许的最大间隔，单位 ms。
 * @return true 表示超时。
 *
 * 调用上下文：20kHz 控制中断周期性检查。
 * 失败行为：timeout_ms 为 0 或从未喂狗时返回 true，保持保守。
 */
bool comm_guard_can_timeout_check(uint32_t now_ms, uint32_t timeout_ms);

/**
 * @brief 日志节流：仅在距上次放行超过最小间隔时放行。
 *
 * @param now_ms 当前单调毫秒时间。
 * @param min_interval_ms 最小放行间隔，单位 ms。
 * @return true 表示允许写日志。
 *
 * 调用上下文：日志任务。
 * 失败行为：min_interval_ms 为 0 时放行并更新基准；被抑制时增加丢弃计数。
 */
bool comm_guard_log_allowed(uint32_t now_ms, uint32_t min_interval_ms);

/**
 * @brief 读取日志丢弃计数。
 *
 * @return 丢弃条数。
 *
 * 调用上下文：诊断。
 * 失败行为：无失败路径。
 */
uint32_t comm_guard_log_dropped(void);

#ifdef __cplusplus
}
#endif

#endif
