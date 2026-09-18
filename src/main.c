/**
 * @file main.c
 * @brief 产品固件入口。
 *
 * 主要接口：main。
 * 依赖关系：依赖 app_init 和 app_scheduler。
 * 关键安全约束：初始化失败也进入安全调度循环，不使用 while(1) 停机吞掉看门狗服务。
 */

#include "app/app_init.h"
#include "app/app_scheduler.h"

/**
 * @brief 启动产品固件并进入 1kHz 调度循环。
 *
 * @return 理论返回 0；当前调度循环不会返回。
 *
 * 调用上下文：复位向量。
 * 失败行为：初始化失败时保持安全输出并继续诊断调度。
 */
int main(void)
{
  (void)app_init();
  app_scheduler_run();
  return 0;
}
