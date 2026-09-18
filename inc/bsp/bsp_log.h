/**
 * @file bsp_log.h
 * @brief UART3 非阻塞调试日志接口。
 *
 * 主要接口：初始化 921600 8N1 UART3 和尝试写入日志。
 * 依赖关系：依赖 AT32F403A USART/GPIO 外设库。
 * 关键安全约束：日志发送不得等待 FIFO、不得在控制中断调用，也不得阻止安全动作。
 */

#ifndef FOC_BSP_LOG_H
#define FOC_BSP_LOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UART3 调试日志口。
 *
 * @return true 表示配置完成。
 *
 * 调用上下文：GPIO 和系统时钟初始化后。
 * 失败行为：配置错误时返回 false，电机安全控制不受日志口影响。
 */
bool bsp_log_init(void);

/**
 * @brief 尝试发送一段日志。
 *
 * @param text UTF-8 或 ASCII 字节序列。
 * @param length 字节数。
 * @return true 表示全部字节已写入发送寄存器。
 *
 * 调用上下文：低优先级日志任务和启动阶段。
 * 失败行为：发送寄存器忙时返回 false；允许已经部分发送，禁止忙等待。
 */
bool bsp_log_write_nonblocking(const char *text, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
