/**
 * @file bsp_flash.h
 * @brief 内部 Flash 只读访问接口。
 *
 * 主要接口：边界检查后的 Flash 读取；v1.0 参数写入路径在后续阶段实现。
 * 依赖关系：仅依赖 C11 标准库。
 * 关键安全约束：代码区读取不得越过 1MiB Flash 边界。
 */

#ifndef FOC_BSP_FLASH_H
#define FOC_BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief AT32F403ACGU7 内部 Flash 基地址。 */
#define BSP_FLASH_BASE_ADDRESS UINT32_C(0x08000000)

/** @brief 当前目标 Flash 容量，单位 byte。 */
#define BSP_FLASH_SIZE_BYTES UINT32_C(0x00100000)

/**
 * @brief 读取内部 Flash。
 *
 * @param address Flash 绝对地址。
 * @param buffer 输出缓冲区。
 * @param length 读取长度，单位 byte。
 * @return true 表示地址范围和参数均有效。
 *
 * 调用上下文：启动参数加载和非实时诊断。
 * 失败行为：越界或空指针返回 false，不读取非法地址。
 */
bool bsp_flash_read(uint32_t address, uint8_t *buffer, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
