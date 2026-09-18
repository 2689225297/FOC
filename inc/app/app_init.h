/**
 * @file app_init.h
 * @brief 产品固件启动初始化接口。
 *
 * 主要接口：建立安全输出、时钟、时基、故障管理、状态机和参数。
 * 依赖关系：依赖 BSP、安全、存储和状态机模块。
 * 关键安全约束：任何初始化或参数校验失败都必须保持 PWM 关闭并进入可诊断状态。
 */

#ifndef FOC_APP_INIT_H
#define FOC_APP_INIT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按冻结顺序初始化产品固件。
 *
 * @return true 表示初始化完成且当前参数允许进入校准流程。
 *
 * 调用上下文：main 开头，进入调度循环之前。
 * 失败行为：失败仍返回并允许调度循环运行诊断；所有功率输出保持关闭。
 */
bool app_init(void);

#ifdef __cplusplus
}
#endif

#endif
