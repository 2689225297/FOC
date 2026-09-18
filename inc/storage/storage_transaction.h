/**
 * @file storage_transaction.h
 * @brief 参数事务写入接口。
 *
 * 主要接口：存储 IO 注入、双槽事务提交、活动槽查询。
 * 依赖关系：依赖 storage_parameter.h 的记录编码与校验、motor_types.h。
 * 关键安全约束：事务写入只在全部轴停机时允许；失败必须保持旧参数有效或
 *              明确回退到未标定安全默认，绝不部分更新。
 */

#ifndef FOC_STORAGE_TRANSACTION_H
#define FOC_STORAGE_TRANSACTION_H

#include <stdbool.h>
#include <stdint.h>

#include "motor/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 存储读取函数指针。
 *
 * @param address Flash 绝对地址。
 * @param buffer 输出缓冲区。
 * @param length 读取长度，单位 byte。
 * @return true 表示读取成功。
 */
typedef bool (*storage_transaction_read_fn)(uint32_t address, uint8_t *buffer, uint32_t length);

/**
 * @brief 存储扇区擦除函数指针。
 *
 * @param address 目标扇区内任一地址。
 * @return true 表示擦除成功。
 */
typedef bool (*storage_transaction_erase_fn)(uint32_t address);

/**
 * @brief 存储编程函数指针。
 *
 * @param address 目标绝对地址。
 * @param buffer 输入数据。
 * @param length 长度，单位 byte。
 * @return true 表示编程成功。
 */
typedef bool (*storage_transaction_program_fn)(uint32_t address,
                                               const uint8_t *buffer,
                                               uint32_t length);

/**
 * @brief 存储 IO 注入接口，由 BSP 或主机测试提供。
 */
typedef struct {
  storage_transaction_read_fn read;      /**< 读取。 */
  storage_transaction_erase_fn erase;    /**< 擦除地址所在扇区。 */
  storage_transaction_program_fn program;/**< 编程。 */
} storage_transaction_io_t;

/**
 * @brief 事务提交结果。
 *
 * 取值范围：OK 表示新记录已完整落盘并通过回读校验。
 */
typedef enum {
  STORAGE_TX_OK = 0,          /**< 新记录写入并回读校验成功。 */
  STORAGE_TX_REJECTED_BUSY,   /**< 轴仍在运行，拒绝写入。 */
  STORAGE_TX_REJECTED_PARAM,  /**< 参数未通过运行校验，拒绝写入。 */
  STORAGE_TX_FAILED,          /**< 擦写或回读校验失败，调用方必须回退安全默认。 */
  STORAGE_TX_IO_INVALID       /**< IO 接口或参数非法。 */
} storage_transaction_status_t;

/**
 * @brief 绑定存储 IO 并探测活动槽。
 *
 * @param io 存储 IO 接口，必须非空且三个函数指针齐全。
 * @return 无返回值。
 *
 * 调用上下文：系统初始化。
 * 失败行为：IO 非法时记录内部无效标志，后续 commit 返回 STORAGE_TX_IO_INVALID。
 */
void storage_transaction_init(const storage_transaction_io_t *io);

/**
 * @brief 以事务方式提交新参数。
 *
 * 流程：校验停机与参数有效性 → 计算新序号 → 擦除参数扇区 → 编程新记录到目标槽
 * → 编程旧有效记录到另一槽 → 回读两槽校验。
 *
 * @param parameters 待提交参数。
 * @param axes_stopped true 表示全部轴已停机，允许写入。
 * @param new_sequence 输出新记录序号；可为 NULL。
 * @return 事务结果。
 *
 * 调用上下文：1kHz 参数管理任务。
 * 失败行为：任一步失败返回 STORAGE_TX_FAILED；调用方必须保持安全停机状态，
 *           禁止继续使用旧参数运行电机。
 */
storage_transaction_status_t storage_transaction_commit(const parameter_set_t *parameters,
                                                        bool axes_stopped,
                                                        uint32_t *new_sequence);

/**
 * @brief 读取当前活动槽序号。
 *
 * @return 活动槽序号；无有效记录返回 0。
 *
 * 调用上下文：诊断。
 * 失败行为：无失败路径。
 */
uint32_t storage_transaction_active_sequence(void);

/**
 * @brief 读取当前活动槽地址。
 *
 * @return 活动槽地址。
 *
 * 调用上下文：诊断。
 * 失败行为：无失败路径。
 */
uint32_t storage_transaction_active_slot_address(void);

#ifdef __cplusplus
}
#endif

#endif
