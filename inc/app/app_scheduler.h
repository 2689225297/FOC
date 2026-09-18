/**
 * @file app_scheduler.h
 * @brief 阶段 0 裸机调度循环接口。
 *
 * 主要接口：运行 1kHz 状态机、自检和看门狗服务。
 * 依赖关系：依赖 BSP 时基、状态机、自检和看门狗模块。
 * 关键安全约束：调度循环不得使能功率；FreeRTOS 接入后任务职责必须保持等价。
 */

#ifndef FOC_APP_SCHEDULER_H
#define FOC_APP_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 执行一次 1kHz 调度步骤。
 *
 * @param timestamp_ms 当前毫秒时间。
 * @return 无返回值。
 *
 * 调用上下文：裸机主循环和主机调度测试。
 * 失败行为：自检失败时只提交 FAULT 事件，不直接修改状态。
 */
void app_scheduler_step(uint32_t timestamp_ms);

/**
 * @brief 运行不会返回的裸机调度循环。
 *
 * @return 无返回值。
 *
 * 调用上下文：main 初始化完成后。
 * 失败行为：参数或硬件故障状态下持续保持安全输出和诊断。
 */
void app_scheduler_run(void);

#ifdef __cplusplus
}
#endif

#endif
