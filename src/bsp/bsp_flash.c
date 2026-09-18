/**
 * @file bsp_flash.c
 * @brief 内部 Flash 只读访问实现。
 *
 * 主要接口：bsp_flash_read。
 * 依赖关系：Flash 可直接寻址读取，本阶段不需要厂商擦写库。
 * 关键安全约束：参数区仍在 Flash 地址范围内，必须先校验范围再复制。
 */

#include "bsp/bsp_flash.h"

#include <string.h>

/**
 * @brief 读取内部 Flash。
 *
 * @param address Flash 绝对地址。
 * @param buffer 输出缓冲区。
 * @param length 读取长度。
 * @return true 表示读取成功。
 *
 * 调用上下文：启动和非实时诊断。
 * 失败行为：非法地址、空指针或长度溢出返回 false。
 */
bool bsp_flash_read(uint32_t address, uint8_t *buffer, uint32_t length)
{
  uint32_t flash_end;
  uint32_t request_end;

  if ((buffer == 0) || (length == 0u)) {
    return false;
  }

  flash_end = BSP_FLASH_BASE_ADDRESS + BSP_FLASH_SIZE_BYTES;
  if ((address < BSP_FLASH_BASE_ADDRESS) || (address >= flash_end)) {
    return false;
  }

  if (length > (flash_end - address)) {
    return false;
  }

  request_end = address + length;
  if (request_end < address) {
    return false;
  }

  memcpy(buffer, (const void *)address, length);
  return true;
}
