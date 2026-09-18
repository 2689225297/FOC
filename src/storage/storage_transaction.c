/**
 * @file storage_transaction.c
 * @brief 参数双槽事务写入实现。
 *
 * 事务语义：新参数写入目标槽并回读校验通过后，旧有效记录才保留到另一槽；
 * 任意一步失败即返回 FAILED，调用方必须回退到未标定安全默认并保持停机。
 * 擦除接口按"地址所在扇区"提供，允许 A/B 槽位于同一扇区的存储布局。
 */

#include "storage/storage_transaction.h"

#include <string.h>

#include "storage/storage_parameter.h"

/** @brief 内部暂存缓冲区长度。 */
#define STORAGE_TX_RECORD_BUFFER_SIZE STORAGE_PARAMETER_RECORD_SIZE

/** @brief 事务层记录的存储状态。 */
typedef struct {
  storage_transaction_io_t io;  /**< 存储 IO 接口。 */
  bool io_valid;                /**< IO 是否已绑定且可用。 */
  uint32_t active_sequence;     /**< 当前活动槽序号。 */
  uint32_t active_slot;         /**< 当前活动槽地址。 */
  bool has_active;              /**< 是否存在有效活动记录。 */
} storage_transaction_state_t;

/** @brief 事务层静态状态。 */
static storage_transaction_state_t g_tx;

/** @brief 两个参数槽地址。 */
static const uint32_t g_slot_addresses[2u] = {
  STORAGE_PARAMETER_SLOT_A_ADDRESS,
  STORAGE_PARAMETER_SLOT_B_ADDRESS
};

/**
 * @brief 从指定槽读取并解析记录。
 *
 * @param slot_index 槽序号 0 或 1。
 * @param parameters 输出参数。
 * @param sequence 输出序号。
 * @return true 表示记录有效。
 */
static bool storage_tx_read_slot(uint32_t slot_index,
                                 parameter_set_t *parameters,
                                 uint32_t *sequence)
{
  uint8_t record[STORAGE_TX_RECORD_BUFFER_SIZE];
  uint32_t parsed_sequence;

  if ((slot_index >= 2u) || (parameters == 0) || (sequence == 0)) {
    return false;
  }

  if (!g_tx.io.read(g_slot_addresses[slot_index], record, sizeof(record))) {
    return false;
  }

  if (!storage_parameter_parse_record(record, sizeof(record), parameters, &parsed_sequence)) {
    return false;
  }

  *parameters = *parameters;
  *sequence = parsed_sequence;
  return true;
}

/**
 * @brief 探测两个槽并选择序号较大的有效记录作为活动槽。
 *
 * @return 无返回值。
 *
 * 调用上下文：storage_transaction_init。
 * 失败行为：无失败路径；无有效记录时 has_active 保持 false。
 */
static void storage_tx_probe_slots(void)
{
  parameter_set_t candidate;
  uint32_t sequence;
  uint32_t index;
  bool found = false;

  g_tx.has_active = false;
  g_tx.active_sequence = 0u;
  g_tx.active_slot = g_slot_addresses[0u];

  for (index = 0u; index < 2u; ++index) {
    if (storage_tx_read_slot(index, &candidate, &sequence)) {
      if (!found || ((int32_t)(sequence - g_tx.active_sequence) > 0)) {
        g_tx.active_sequence = sequence;
        g_tx.active_slot = g_slot_addresses[index];
        found = true;
      }
    }
  }

  g_tx.has_active = found;
}

/**
 * @brief 绑定存储 IO 并探测活动槽。
 *
 * @param io 存储 IO 接口。
 * @return 无返回值。
 */
void storage_transaction_init(const storage_transaction_io_t *io)
{
  memset(&g_tx, 0, sizeof(g_tx));

  if ((io == 0) || (io->read == 0) || (io->erase == 0) || (io->program == 0)) {
    g_tx.io_valid = false;
    return;
  }

  g_tx.io = *io;
  g_tx.io_valid = true;
  storage_tx_probe_slots();
}

/**
 * @brief 编程并回读校验一条记录。
 *
 * @param address 目标槽地址。
 * @param parameters 待写入参数。
 * @param sequence 待写入序号。
 * @return true 表示编程且回读校验成功。
 */
static bool storage_tx_program_slot(uint32_t address,
                                    const parameter_set_t *parameters,
                                    uint32_t sequence)
{
  uint8_t record[STORAGE_TX_RECORD_BUFFER_SIZE];
  uint8_t readback[STORAGE_TX_RECORD_BUFFER_SIZE];
  parameter_set_t parsed;
  uint32_t parsed_sequence;

  if (!storage_parameter_build_record(parameters, sequence, record, sizeof(record))) {
    return false;
  }

  if (!g_tx.io.program(address, record, sizeof(record))) {
    return false;
  }

  if (!g_tx.io.read(address, readback, sizeof(readback))) {
    return false;
  }

  if (!storage_parameter_parse_record(readback, sizeof(readback), &parsed, &parsed_sequence)) {
    return false;
  }

  if (parsed_sequence != sequence) {
    return false;
  }

  return true;
}

/**
 * @brief 以事务方式提交新参数。
 *
 * @param parameters 待提交参数。
 * @param axes_stopped true 表示全部轴已停机。
 * @param new_sequence 输出新记录序号；可为 NULL。
 * @return 事务结果。
 */
storage_transaction_status_t storage_transaction_commit(const parameter_set_t *parameters,
                                                        bool axes_stopped,
                                                        uint32_t *new_sequence)
{
  parameter_set_t preserved;
  uint32_t preserved_sequence = 0u;
  uint32_t target_slot_index;
  uint32_t preserved_slot_index;
  uint32_t new_sequence_value;
  bool preserved_valid = false;

  if (!g_tx.io_valid) {
    return STORAGE_TX_IO_INVALID;
  }

  if (parameters == 0) {
    return STORAGE_TX_IO_INVALID;
  }

  if (!axes_stopped) {
    return STORAGE_TX_REJECTED_BUSY;
  }

  if (!storage_parameter_is_operational(parameters)) {
    return STORAGE_TX_REJECTED_PARAM;
  }

  /* 选择非活动槽作为目标槽；无有效记录时目标槽为 A 槽。 */
  if (g_tx.has_active && (g_tx.active_slot == g_slot_addresses[0u])) {
    target_slot_index = 1u;
    preserved_slot_index = 0u;
  } else {
    target_slot_index = 0u;
    preserved_slot_index = 1u;
  }

  if (g_tx.has_active) {
    preserved_valid = storage_tx_read_slot(preserved_slot_index, &preserved, &preserved_sequence);
    if (!preserved_valid) {
      return STORAGE_TX_FAILED;
    }
  }

  new_sequence_value = g_tx.has_active ? (g_tx.active_sequence + 1u) : 1u;

  /* 擦除目标地址所在扇区（覆盖两槽），然后先写新记录，再写旧记录。 */
  if (!g_tx.io.erase(g_slot_addresses[target_slot_index])) {
    return STORAGE_TX_FAILED;
  }

  if (!storage_tx_program_slot(g_slot_addresses[target_slot_index],
                               parameters, new_sequence_value)) {
    return STORAGE_TX_FAILED;
  }

  if (g_tx.has_active &&
      !storage_tx_program_slot(g_slot_addresses[preserved_slot_index],
                               &preserved, preserved_sequence)) {
    return STORAGE_TX_FAILED;
  }

  g_tx.active_sequence = new_sequence_value;
  g_tx.active_slot = g_slot_addresses[target_slot_index];
  g_tx.has_active = true;

  if (new_sequence != 0) {
    *new_sequence = new_sequence_value;
  }

  return STORAGE_TX_OK;
}

/**
 * @brief 读取当前活动槽序号。
 *
 * @return 活动槽序号；无有效记录返回 0。
 */
uint32_t storage_transaction_active_sequence(void)
{
  return g_tx.has_active ? g_tx.active_sequence : 0u;
}

/**
 * @brief 读取当前活动槽地址。
 *
 * @return 活动槽地址。
 */
uint32_t storage_transaction_active_slot_address(void)
{
  return g_tx.active_slot;
}
