/**
 * @file storage_parameter.c
 * @brief 参数记录固定格式编码、校验和安全默认值实现。
 *
 * 主要接口：storage_parameter_build_record、storage_parameter_parse_record、
 * storage_parameter_defaults、storage_parameter_is_operational。
 * 依赖关系：依赖 motor_types.h、storage_crc32.h。
 * 关键安全约束：禁止直接按结构体内存布局写 Flash；所有多字节字段使用固定小端编码。
 */

#include "storage/storage_parameter.h"
#include "storage/storage_crc32.h"

#include <float.h>
#include <string.h>

/** @brief 参数记录头字段偏移，单位 byte。 */
enum {
  PARAM_HEADER_MAGIC_OFFSET = 0,
  PARAM_HEADER_SCHEMA_OFFSET = 4,
  PARAM_HEADER_HEADER_SIZE_OFFSET = 6,
  PARAM_HEADER_PAYLOAD_SIZE_OFFSET = 8,
  PARAM_HEADER_FLAGS_OFFSET = 10,
  PARAM_HEADER_SEQUENCE_OFFSET = 12,
  PARAM_HEADER_PARAMETER_VERSION_OFFSET = 16,
  PARAM_HEADER_RESERVED0_OFFSET = 18,
  PARAM_HEADER_PAYLOAD_CRC_OFFSET = 20,
  PARAM_HEADER_HEADER_CRC_OFFSET = 24,
  PARAM_HEADER_RESERVED1_OFFSET = 28
};

/** @brief 参数正文内连续 float 字段从偏移 8 开始。 */
enum {
  PARAM_PAYLOAD_SCHEMA_OFFSET = 0,
  PARAM_PAYLOAD_PARAMETER_VERSION_OFFSET = 2,
  PARAM_PAYLOAD_POLE_PAIRS_OFFSET = 4,
  PARAM_PAYLOAD_CALIBRATION_MASK_OFFSET = 5,
  PARAM_PAYLOAD_FLOAT_BASE = 8
};

/**
 * @brief 写入小端 16 位整数。
 *
 * @param destination 输出地址，至少 2 byte。
 * @param value 待写入值。
 * @return 无返回值。
 *
 * 调用上下文：参数记录构建。
 * 失败行为：调用方必须保证 destination 有效；本函数不检查空指针。
 */
static void parameter_put_u16(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)(value & UINT16_C(0x00FF));
  destination[1] = (uint8_t)((value >> 8u) & UINT16_C(0x00FF));
}

/**
 * @brief 写入小端 32 位整数。
 *
 * @param destination 输出地址，至少 4 byte。
 * @param value 待写入值。
 * @return 无返回值。
 *
 * 调用上下文：参数记录构建。
 * 失败行为：调用方必须保证 destination 有效；本函数不检查空指针。
 */
static void parameter_put_u32(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)(value & UINT32_C(0x000000FF));
  destination[1] = (uint8_t)((value >> 8u) & UINT32_C(0x000000FF));
  destination[2] = (uint8_t)((value >> 16u) & UINT32_C(0x000000FF));
  destination[3] = (uint8_t)((value >> 24u) & UINT32_C(0x000000FF));
}

/**
 * @brief 读取小端 16 位整数。
 *
 * @param source 输入地址，至少 2 byte。
 * @return 读取值。
 *
 * 调用上下文：参数记录解析。
 * 失败行为：调用方必须保证 source 有效；本函数不检查空指针。
 */
static uint16_t parameter_get_u16(const uint8_t *source)
{
  return (uint16_t)((uint16_t)source[0] |
                    ((uint16_t)source[1] << 8u));
}

/**
 * @brief 读取小端 32 位整数。
 *
 * @param source 输入地址，至少 4 byte。
 * @return 读取值。
 *
 * 调用上下文：参数记录解析。
 * 失败行为：调用方必须保证 source 有效；本函数不检查空指针。
 */
static uint32_t parameter_get_u32(const uint8_t *source)
{
  return (uint32_t)source[0] |
         ((uint32_t)source[1] << 8u) |
         ((uint32_t)source[2] << 16u) |
         ((uint32_t)source[3] << 24u);
}

/**
 * @brief 按 IEEE 754 单精度位模式写入 float。
 *
 * @param destination 输出地址，至少 4 byte。
 * @param value 待写入值。
 * @return 无返回值。
 *
 * 调用上下文：参数记录构建。
 * 失败行为：调用方必须保证目标工具链 float 为 32 位；启动静态检查会验证该前提。
 */
static void parameter_put_float(uint8_t *destination, float value)
{
  uint32_t bits;

  memcpy(&bits, &value, sizeof(bits));
  parameter_put_u32(destination, bits);
}

/**
 * @brief 按 IEEE 754 单精度位模式读取 float。
 *
 * @param source 输入地址，至少 4 byte。
 * @return 读取并还原的 float。
 *
 * 调用上下文：参数记录解析。
 * 失败行为：调用方必须保证目标工具链 float 为 32 位。
 */
static float parameter_get_float(const uint8_t *source)
{
  uint32_t bits;
  float value;

  bits = parameter_get_u32(source);
  memcpy(&value, &bits, sizeof(value));
  return value;
}

/**
 * @brief 判断单精度浮点是否为有限数。
 *
 * @param value 输入值。
 * @return true 表示不是 NaN 且绝对值不超过 FLT_MAX。
 *
 * 调用上下文：参数范围校验。
 * 失败行为：无失败路径，不修改输入。
 */
static bool parameter_float_is_finite(float value)
{
  return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/**
 * @brief 按冻结顺序编码参数正文。
 *
 * @param parameters 输入参数。
 * @param payload 输出 128 byte 正文。
 * @return 无返回值。
 *
 * 调用上下文：storage_parameter_build_record。
 * 失败行为：调用方必须保证参数和缓冲区有效。
 */
static void parameter_encode_payload(const parameter_set_t *parameters, uint8_t *payload)
{
  uint32_t offset;
  uint32_t axis;

  parameter_put_u16(&payload[PARAM_PAYLOAD_SCHEMA_OFFSET], parameters->schema_version);
  parameter_put_u16(&payload[PARAM_PAYLOAD_PARAMETER_VERSION_OFFSET], parameters->parameter_version);
  payload[PARAM_PAYLOAD_POLE_PAIRS_OFFSET] = parameters->pole_pairs;
  payload[PARAM_PAYLOAD_CALIBRATION_MASK_OFFSET] = parameters->calibration_valid_mask;
  parameter_put_u16(&payload[6], UINT16_C(0));

  offset = PARAM_PAYLOAD_FLOAT_BASE;
  parameter_put_float(&payload[offset], parameters->rs_ohm);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->ld_h);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->lq_h);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->flux_wb);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->inertia_kg_m2);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->torque_constant_nm_per_a);
  offset += 4u;

  for (axis = 0u; axis < MOTOR_AXIS_COUNT; ++axis) {
    parameter_put_float(&payload[offset], parameters->current_calibration[axis].current_gain_a_per_v);
    offset += 4u;
    parameter_put_float(&payload[offset], parameters->current_calibration[axis].current_zero_offset_v);
    offset += 4u;
    parameter_put_float(&payload[offset], parameters->current_calibration[axis].current_polarity);
    offset += 4u;
  }

  parameter_put_float(&payload[offset], parameters->configured_bus_voltage_v);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->continuous_current_a);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->peak_current_a);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->immediate_current_a);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->max_speed_rpm);
  offset += 4u;
  parameter_put_float(&payload[offset], parameters->over_speed_fault_rpm);
}

/**
 * @brief 按冻结顺序解码参数正文。
 *
 * @param payload 输入 128 byte 正文。
 * @param parameters 输出参数。
 * @return 无返回值。
 *
 * 调用上下文：storage_parameter_parse_record。
 * 失败行为：调用方必须保证缓冲区和输出结构体有效。
 */
static void parameter_decode_payload(const uint8_t *payload, parameter_set_t *parameters)
{
  uint32_t offset;
  uint32_t axis;

  parameters->schema_version = parameter_get_u16(&payload[PARAM_PAYLOAD_SCHEMA_OFFSET]);
  parameters->parameter_version = parameter_get_u16(&payload[PARAM_PAYLOAD_PARAMETER_VERSION_OFFSET]);
  parameters->pole_pairs = payload[PARAM_PAYLOAD_POLE_PAIRS_OFFSET];
  parameters->calibration_valid_mask = payload[PARAM_PAYLOAD_CALIBRATION_MASK_OFFSET];
  parameters->reserved0 = parameter_get_u16(&payload[6]);

  offset = PARAM_PAYLOAD_FLOAT_BASE;
  parameters->rs_ohm = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->ld_h = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->lq_h = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->flux_wb = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->inertia_kg_m2 = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->torque_constant_nm_per_a = parameter_get_float(&payload[offset]);
  offset += 4u;

  for (axis = 0u; axis < MOTOR_AXIS_COUNT; ++axis) {
    parameters->current_calibration[axis].current_gain_a_per_v = parameter_get_float(&payload[offset]);
    offset += 4u;
    parameters->current_calibration[axis].current_zero_offset_v = parameter_get_float(&payload[offset]);
    offset += 4u;
    parameters->current_calibration[axis].current_polarity = parameter_get_float(&payload[offset]);
    offset += 4u;
  }

  parameters->configured_bus_voltage_v = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->continuous_current_a = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->peak_current_a = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->immediate_current_a = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->max_speed_rpm = parameter_get_float(&payload[offset]);
  offset += 4u;
  parameters->over_speed_fault_rpm = parameter_get_float(&payload[offset]);
}

/**
 * @brief 校验参数标定有效位、有限值和运行范围。
 *
 * @param parameters 输入参数。
 * @return true 表示允许进入带功率闭环。
 *
 * 调用上下文：参数加载和模式切换前。
 * 失败行为：NULL 或任意范围错误返回 false。
 */
bool storage_parameter_is_operational(const parameter_set_t *parameters)
{
  uint32_t axis;
  uint8_t required_mask;

  if (parameters == 0) {
    return false;
  }

  required_mask = STORAGE_CALIBRATION_VALID_A |
                  STORAGE_CALIBRATION_VALID_B |
                  STORAGE_CALIBRATION_VALID_MOTOR;

  if ((parameters->schema_version != MOTOR_PARAMETER_SCHEMA_VERSION) ||
      (parameters->parameter_version != MOTOR_PARAMETER_VERSION) ||
      (parameters->reserved0 != 0u) ||
      (parameters->pole_pairs < 1u) ||
      (parameters->pole_pairs > 32u) ||
      ((parameters->calibration_valid_mask & required_mask) != required_mask)) {
    return false;
  }

  if (!parameter_float_is_finite(parameters->rs_ohm) ||
      !parameter_float_is_finite(parameters->ld_h) ||
      !parameter_float_is_finite(parameters->lq_h) ||
      !parameter_float_is_finite(parameters->flux_wb) ||
      !parameter_float_is_finite(parameters->inertia_kg_m2) ||
      !parameter_float_is_finite(parameters->torque_constant_nm_per_a) ||
      (parameters->rs_ohm <= 0.0f) ||
      (parameters->ld_h <= 0.0f) ||
      (parameters->lq_h <= 0.0f) ||
      (parameters->flux_wb <= 0.0f) ||
      (parameters->inertia_kg_m2 <= 0.0f) ||
      (parameters->torque_constant_nm_per_a <= 0.0f)) {
    return false;
  }

  for (axis = 0u; axis < MOTOR_AXIS_COUNT; ++axis) {
    const current_calibration_t *calibration = &parameters->current_calibration[axis];

    if (!parameter_float_is_finite(calibration->current_gain_a_per_v) ||
        !parameter_float_is_finite(calibration->current_zero_offset_v) ||
        !parameter_float_is_finite(calibration->current_polarity) ||
        (calibration->current_gain_a_per_v <= 0.0f) ||
        (calibration->current_zero_offset_v < 0.0f) ||
        (calibration->current_zero_offset_v > 3.3f) ||
        ((calibration->current_polarity != 1.0f) &&
         (calibration->current_polarity != -1.0f))) {
      return false;
    }
  }

  if (!parameter_float_is_finite(parameters->configured_bus_voltage_v) ||
      !parameter_float_is_finite(parameters->continuous_current_a) ||
      !parameter_float_is_finite(parameters->peak_current_a) ||
      !parameter_float_is_finite(parameters->immediate_current_a) ||
      !parameter_float_is_finite(parameters->max_speed_rpm) ||
      !parameter_float_is_finite(parameters->over_speed_fault_rpm) ||
      (parameters->configured_bus_voltage_v < 8.0f) ||
      (parameters->configured_bus_voltage_v > 20.0f) ||
      (parameters->continuous_current_a <= 0.0f) ||
      (parameters->peak_current_a < parameters->continuous_current_a) ||
      (parameters->immediate_current_a < parameters->peak_current_a) ||
      (parameters->max_speed_rpm <= 0.0f) ||
      (parameters->over_speed_fault_rpm <= parameters->max_speed_rpm)) {
    return false;
  }

  return true;
}

/**
 * @brief 填充未标定安全默认参数。
 *
 * @param parameters 输出参数。
 * @return 无返回值。
 *
 * 调用上下文：首次启动诊断和记录模板生成。
 * 失败行为：NULL 输入直接返回；默认 calibration_valid_mask 为 0。
 */
void storage_parameter_defaults(parameter_set_t *parameters)
{
  uint32_t axis;

  if (parameters == 0) {
    return;
  }

  memset(parameters, 0, sizeof(*parameters));
  parameters->schema_version = MOTOR_PARAMETER_SCHEMA_VERSION;
  parameters->parameter_version = MOTOR_PARAMETER_VERSION;
  parameters->pole_pairs = 7u;
  parameters->calibration_valid_mask = 0u;

  for (axis = 0u; axis < MOTOR_AXIS_COUNT; ++axis) {
    parameters->current_calibration[axis].current_gain_a_per_v = 20.0f;
    parameters->current_calibration[axis].current_zero_offset_v = 1.65f;
    parameters->current_calibration[axis].current_polarity = 1.0f;
  }

  parameters->configured_bus_voltage_v = 12.0f;
  parameters->continuous_current_a = 12.0f;
  parameters->peak_current_a = 18.0f;
  parameters->immediate_current_a = 20.0f;
  parameters->max_speed_rpm = 7000.0f;
  parameters->over_speed_fault_rpm = 7500.0f;
}

/**
 * @brief 构建固定长度参数 Flash 记录。
 *
 * @param parameters 输入参数。
 * @param sequence 输入记录序号。
 * @param record 输出缓冲区。
 * @param capacity 输出容量。
 * @return true 表示记录构建成功。
 *
 * 调用上下文：停机状态的参数存储任务。
 * 失败行为：容量不足或前置参数非法时返回 false，不执行 Flash 写入。
 */
bool storage_parameter_build_record(const parameter_set_t *parameters,
                                    uint32_t sequence,
                                    uint8_t *record,
                                    uint32_t capacity)
{
  uint8_t header_without_crc[STORAGE_PARAMETER_HEADER_SIZE];
  uint32_t header_crc;

  if ((parameters == 0) || (record == 0) ||
      (capacity < STORAGE_PARAMETER_RECORD_SIZE) ||
      (parameters->schema_version != MOTOR_PARAMETER_SCHEMA_VERSION) ||
      (parameters->parameter_version != MOTOR_PARAMETER_VERSION)) {
    return false;
  }

  memset(record, 0xFF, capacity);
  memset(header_without_crc, 0, sizeof(header_without_crc));

  parameter_put_u32(&header_without_crc[PARAM_HEADER_MAGIC_OFFSET], STORAGE_PARAMETER_MAGIC);
  parameter_put_u16(&header_without_crc[PARAM_HEADER_SCHEMA_OFFSET], parameters->schema_version);
  parameter_put_u16(&header_without_crc[PARAM_HEADER_HEADER_SIZE_OFFSET],
                    (uint16_t)STORAGE_PARAMETER_HEADER_SIZE);
  parameter_put_u16(&header_without_crc[PARAM_HEADER_PAYLOAD_SIZE_OFFSET],
                    (uint16_t)STORAGE_PARAMETER_PAYLOAD_SIZE);
  parameter_put_u16(&header_without_crc[PARAM_HEADER_FLAGS_OFFSET],
                    STORAGE_PARAMETER_FLAG_VALID);
  parameter_put_u32(&header_without_crc[PARAM_HEADER_SEQUENCE_OFFSET], sequence);
  parameter_put_u16(&header_without_crc[PARAM_HEADER_PARAMETER_VERSION_OFFSET],
                    parameters->parameter_version);

  parameter_encode_payload(parameters, &record[STORAGE_PARAMETER_HEADER_SIZE]);
  parameter_put_u32(&header_without_crc[PARAM_HEADER_PAYLOAD_CRC_OFFSET],
                    storage_crc32_calculate(&record[STORAGE_PARAMETER_HEADER_SIZE],
                                            STORAGE_PARAMETER_PAYLOAD_SIZE));

  header_crc = storage_crc32_calculate(header_without_crc, STORAGE_PARAMETER_HEADER_SIZE);
  parameter_put_u32(&header_without_crc[PARAM_HEADER_HEADER_CRC_OFFSET], header_crc);
  memcpy(record, header_without_crc, STORAGE_PARAMETER_HEADER_SIZE);
  return true;
}

/**
 * @brief 校验并解析固定长度参数记录。
 *
 * @param record 输入记录。
 * @param length 输入长度。
 * @param parameters 输出参数。
 * @param sequence 可选输出序号。
 * @return true 表示头、正文、版本和保留字段均合法。
 *
 * 调用上下文：启动加载和非实时维护。
 * 失败行为：任一校验失败返回 false，输出结构体内容不得用于运行。
 */
bool storage_parameter_parse_record(const uint8_t *record,
                                    uint32_t length,
                                    parameter_set_t *parameters,
                                    uint32_t *sequence)
{
  uint8_t header_without_crc[STORAGE_PARAMETER_HEADER_SIZE];
  uint32_t stored_header_crc;
  uint32_t calculated_header_crc;
  uint32_t stored_payload_crc;
  uint32_t calculated_payload_crc;
  uint16_t flags;
  uint16_t schema_version;
  uint16_t parameter_version;
  uint16_t reserved0;
  uint16_t reserved1;

  if ((record == 0) || (parameters == 0) ||
      (length < STORAGE_PARAMETER_RECORD_SIZE)) {
    return false;
  }

  if ((parameter_get_u32(&record[PARAM_HEADER_MAGIC_OFFSET]) != STORAGE_PARAMETER_MAGIC) ||
      (parameter_get_u16(&record[PARAM_HEADER_HEADER_SIZE_OFFSET]) != STORAGE_PARAMETER_HEADER_SIZE) ||
      (parameter_get_u16(&record[PARAM_HEADER_PAYLOAD_SIZE_OFFSET]) != STORAGE_PARAMETER_PAYLOAD_SIZE)) {
    return false;
  }

  flags = parameter_get_u16(&record[PARAM_HEADER_FLAGS_OFFSET]);
  if ((flags & STORAGE_PARAMETER_FLAG_VALID) == 0u) {
    return false;
  }

  schema_version = parameter_get_u16(&record[PARAM_HEADER_SCHEMA_OFFSET]);
  parameter_version = parameter_get_u16(&record[PARAM_HEADER_PARAMETER_VERSION_OFFSET]);
  reserved0 = parameter_get_u16(&record[PARAM_HEADER_RESERVED0_OFFSET]);
  reserved1 = parameter_get_u16(&record[PARAM_HEADER_RESERVED1_OFFSET]);

  if ((schema_version != MOTOR_PARAMETER_SCHEMA_VERSION) ||
      (parameter_version != MOTOR_PARAMETER_VERSION) ||
      (reserved0 != 0u) || (reserved1 != 0u)) {
    return false;
  }

  memcpy(header_without_crc, record, sizeof(header_without_crc));
  stored_header_crc = parameter_get_u32(&header_without_crc[PARAM_HEADER_HEADER_CRC_OFFSET]);
  parameter_put_u32(&header_without_crc[PARAM_HEADER_HEADER_CRC_OFFSET], 0u);
  calculated_header_crc = storage_crc32_calculate(header_without_crc,
                                                  STORAGE_PARAMETER_HEADER_SIZE);
  if (stored_header_crc != calculated_header_crc) {
    return false;
  }

  stored_payload_crc = parameter_get_u32(&record[PARAM_HEADER_PAYLOAD_CRC_OFFSET]);
  calculated_payload_crc = storage_crc32_calculate(&record[STORAGE_PARAMETER_HEADER_SIZE],
                                                   STORAGE_PARAMETER_PAYLOAD_SIZE);
  if (stored_payload_crc != calculated_payload_crc) {
    return false;
  }

  parameter_decode_payload(&record[STORAGE_PARAMETER_HEADER_SIZE], parameters);
  if ((parameters->schema_version != schema_version) ||
      (parameters->parameter_version != parameter_version) ||
      (parameters->reserved0 != 0u)) {
    return false;
  }

  if (sequence != 0) {
    *sequence = parameter_get_u32(&record[PARAM_HEADER_SEQUENCE_OFFSET]);
  }

  return true;
}
