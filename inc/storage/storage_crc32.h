/**
 * @file storage_crc32.h
 * @brief IEEE 802.3 CRC32 计算接口。
 *
 * 主要接口：提供分步 CRC32 更新函数，供参数头和参数正文共用。
 * 依赖关系：仅依赖 C11 标准类型。
 * 关键安全约束：参数记录必须使用固定的反射多项式和初始值，禁止随构建选项变化。
 */

#ifndef FOC_STORAGE_CRC32_H
#define FOC_STORAGE_CRC32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief CRC32 初始值。 */
#define STORAGE_CRC32_INITIAL_VALUE UINT32_C(0xFFFFFFFF)

/** @brief CRC32 反射多项式。 */
#define STORAGE_CRC32_POLYNOMIAL UINT32_C(0xEDB88320)

/** @brief CRC32 最终异或值。 */
#define STORAGE_CRC32_FINAL_XOR UINT32_C(0xFFFFFFFF)

/**
 * @brief 分步更新 CRC32。
 *
 * @param crc 上一次返回的 CRC 中间值；首次调用传 STORAGE_CRC32_INITIAL_VALUE。
 * @param data 输入数据起始地址；length 大于 0 时不得为 NULL。
 * @param length 输入字节数，单位 byte。
 * @return 更新后的 CRC 中间值，尚未执行最终异或。
 *
 * 调用上下文：启动加载、参数构建和主机测试均可调用。
 * 失败行为：输入非法时由调用方负责避免；本函数不阻塞且不分配内存。
 */
uint32_t storage_crc32_update(uint32_t crc, const uint8_t *data, uint32_t length);

/**
 * @brief 一次计算完整 CRC32。
 *
 * @param data 输入数据起始地址；length 大于 0 时不得为 NULL。
 * @param length 输入字节数，单位 byte。
 * @return 已完成初始值和最终异或的 CRC32。
 *
 * 调用上下文：可用于非实时参数校验。
 * 失败行为：空数据返回 0；本函数不阻塞且不分配内存。
 */
uint32_t storage_crc32_calculate(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
