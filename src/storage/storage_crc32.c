/**
 * @file storage_crc32.c
 * @brief IEEE 802.3 CRC32 实现。
 *
 * 主要接口：storage_crc32_update、storage_crc32_calculate。
 * 依赖关系：只依赖 storage_crc32.h。
 * 关键安全约束：参数头和参数正文使用相同算法；算法变化必须升级参数格式。
 */

#include "storage/storage_crc32.h"

/**
 * @brief 分步更新 CRC32。
 *
 * @param crc 上一次返回的 CRC 中间值；首次调用传 0xFFFFFFFF。
 * @param data 输入数据起始地址。
 * @param length 输入字节数。
 * @return 更新后的 CRC 中间值。
 *
 * 调用上下文：可被启动、参数存储和主机测试调用。
 * 失败行为：data 为 NULL 且 length 非零时返回当前 CRC，不读取非法内存。
 */
uint32_t storage_crc32_update(uint32_t crc, const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if ((data == 0) && (length != 0u)) {
    return crc;
  }

  /* 逐位处理可以避免大查表占用，同时保证与参数格式固定绑定。 */
  for (index = 0u; index < length; ++index) {
    uint32_t bit;

    crc ^= (uint32_t)data[index];
    for (bit = 0u; bit < 8u; ++bit) {
      if ((crc & UINT32_C(1)) != 0u) {
        crc = (crc >> 1u) ^ STORAGE_CRC32_POLYNOMIAL;
      } else {
        crc >>= 1u;
      }
    }
  }

  return crc;
}

/**
 * @brief 一次计算完整 CRC32。
 *
 * @param data 输入数据起始地址。
 * @param length 输入字节数。
 * @return 已完成初始值和最终异或的 CRC32。
 *
 * 调用上下文：参数记录构建、校验和主机测试。
 * 失败行为：data 为 NULL 且 length 非零时返回 0。
 */
uint32_t storage_crc32_calculate(const uint8_t *data, uint32_t length)
{
  uint32_t crc;

  if ((data == 0) && (length != 0u)) {
    return 0u;
  }

  crc = storage_crc32_update(STORAGE_CRC32_INITIAL_VALUE, data, length);
  return crc ^ STORAGE_CRC32_FINAL_XOR;
}
