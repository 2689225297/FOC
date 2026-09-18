/**
 * @file storage_parameter.h
 * @brief 参数记录构建、解析和有效性接口。
 *
 * 主要接口：构建固定长度 Flash 记录、解析记录、生成安全默认参数。
 * 依赖关系：依赖 motor_types.h 和 storage_crc32.h。
 * 关键安全约束：CRC、版本、长度、保留字段或标定有效位不满足时，调用方不得使能功率输出。
 */

#ifndef FOC_STORAGE_PARAMETER_H
#define FOC_STORAGE_PARAMETER_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 参数记录魔数，小端存储为字符 FOC1。 */
#define STORAGE_PARAMETER_MAGIC UINT32_C(0x31434F46)

/** @brief 参数记录头长度，单位 byte。 */
#define STORAGE_PARAMETER_HEADER_SIZE ((uint32_t)32u)

/** @brief 参数正文长度，单位 byte。 */
#define STORAGE_PARAMETER_PAYLOAD_SIZE ((uint32_t)128u)

/** @brief 单个参数记录有效长度，单位 byte。 */
#define STORAGE_PARAMETER_RECORD_SIZE \
  (STORAGE_PARAMETER_HEADER_SIZE + STORAGE_PARAMETER_PAYLOAD_SIZE)

/** @brief 参数槽大小，单位 byte，与 Flash 扇区一致。 */
#define STORAGE_PARAMETER_SLOT_SIZE ((uint32_t)2048u)

/** @brief 参数槽 A 起始地址，位于 1MiB Flash 的最后 4KiB 内。 */
#define STORAGE_PARAMETER_SLOT_A_ADDRESS UINT32_C(0x080FF000)

/** @brief 参数槽 B 起始地址，位于 1MiB Flash 的最后 2KiB。 */
#define STORAGE_PARAMETER_SLOT_B_ADDRESS UINT32_C(0x080FF800)

/** @brief record_flags 中的有效记录位。 */
#define STORAGE_PARAMETER_FLAG_VALID UINT16_C(0x0001)

/** @brief calibration_valid_mask 中的 A 路电流标定位。 */
#define STORAGE_CALIBRATION_VALID_A UINT8_C(0x01)

/** @brief calibration_valid_mask 中的 B 路电流标定位。 */
#define STORAGE_CALIBRATION_VALID_B UINT8_C(0x02)

/** @brief calibration_valid_mask 中的电机参数有效位。 */
#define STORAGE_CALIBRATION_VALID_MOTOR UINT8_C(0x04)

/**
 * @brief 把运行期参数编码为固定长度 Flash 记录。
 *
 * @param parameters 输入参数，所有字段必须可编码。
 * @param sequence 参数记录序号，由存储层维护。
 * @param record 输出缓冲区起始地址。
 * @param capacity 输出缓冲区容量，单位 byte，必须不小于 160。
 * @return true 表示构建成功；false 表示参数、指针或容量非法。
 *
 * 调用上下文：只允许在对应轴停止的存储任务中调用。
 * 失败行为：失败时不修改已有 Flash 记录，缓冲区内容可能部分改变。
 */
bool storage_parameter_build_record(const parameter_set_t *parameters,
                                    uint32_t sequence,
                                    uint8_t *record,
                                    uint32_t capacity);

/**
 * @brief 校验并解析固定长度 Flash 记录。
 *
 * @param record 输入记录起始地址。
 * @param length 输入记录长度，单位 byte，至少为 160。
 * @param parameters 输出参数。
 * @param sequence 可选输出序号；允许为 NULL。
 * @return true 表示记录完整且有效；false 表示任一校验失败。
 *
 * 调用上下文：启动加载和非实时参数维护。
 * 失败行为：返回 false 时 parameters 内容不可信，调用方必须进入安全停机。
 */
bool storage_parameter_parse_record(const uint8_t *record,
                                    uint32_t length,
                                    parameter_set_t *parameters,
                                    uint32_t *sequence);

/**
 * @brief 填充未标定的安全默认参数。
 *
 * @param parameters 输出参数地址；不得为 NULL。
 * @return 无返回值。
 *
 * 调用上下文：参数记录不存在或首次烧录时用于构造诊断候选。
 * 失败行为：NULL 输入直接返回；默认值的 calibration_valid_mask 为 0，因此不能运行电机。
 */
void storage_parameter_defaults(parameter_set_t *parameters);

/**
 * @brief 判断参数是否同时满足格式和运行校验。
 *
 * @param parameters 输入参数。
 * @return true 表示电机参数、A/B 电流标定和安全范围均有效。
 *
 * 调用上下文：状态机进入 CALIBRATION 或 IDLE 前。
 * 失败行为：NULL、非有限值、范围错误或标定位缺失均返回 false。
 */
bool storage_parameter_is_operational(const parameter_set_t *parameters);

#ifdef __cplusplus
}
#endif

#endif
