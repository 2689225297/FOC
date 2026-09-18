/**
 * @file test_a4_no_power.h
 * @brief A4 阶段无功率验证用例与证据记录接口。
 *
 * 主要接口：test_a4_no_power_run、test_a4_no_power_run_all、test_a4_no_power_log_report、
 *          test_a4_no_power_check_name。
 * 依赖关系：依赖 bsp_pwm、bsp_current_sense、bsp_drv8323、bsp_time、bsp_log、motor_current、
 *          safety 和 storage_parameter；不依赖任何测试框架和动态分配。
 * 关键安全约束：全部用例只允许在功率输出禁用、母线无功率的条件下执行；用例本身不使能
 *              任何功率输出，nFAULT 用例会向 AXIS_B 锁存 FAULT_DRV8323_NFAULT，
 *              执行后必须复位或走人工恢复流程。
 */

#ifndef FOC_TEST_A4_NO_POWER_H
#define FOC_TEST_A4_NO_POWER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief nFAULT 快速路径关闭 PWM 的时间上限，单位 ms，对应 A4 闸门。 */
#define A4_NO_POWER_NFAULT_DEADLINE_MS ((uint32_t)1u)

/** @brief 证据日志行缓冲长度，单位 byte。 */
#define A4_NO_POWER_LOG_LINE_SIZE ((uint32_t)72u)

/**
 * @brief 无功率验证项编号。
 *
 * 取值范围：越界的 A4_NO_POWER_CHECK_COUNT 只作为掩码上限，不作为用例执行。
 */
typedef enum {
  A4_NO_POWER_CHECK_PB12_LOW = 0,        /**< PB12 上电默认低。 */
  A4_NO_POWER_CHECK_A_OUTPUT_INVALID,    /**< A 路无有效 PWM 输出。 */
  A4_NO_POWER_CHECK_B_BRIDGE_HIZ,        /**< B 路三相桥臂处于 Hi-Z。 */
  A4_NO_POWER_CHECK_PWM_TIMING,          /**< 20kHz 中心对齐、死区与最小脉宽。 */
  A4_NO_POWER_CHECK_ADC_TRIGGER,         /**< TMR3/TMR4 同步与 ADC 注入触发已装订。 */
  A4_NO_POWER_CHECK_SAMPLE_WINDOW,       /**< 注入采样窗口不低于下限。 */
  A4_NO_POWER_CHECK_CURRENT_CALIBRATION, /**< 电流量程、增益误差和零偏门槛。 */
  A4_NO_POWER_CHECK_OFFSET_CAPTURE,      /**< 零偏采集离散度检查。 */
  A4_NO_POWER_CHECK_PARAM_CRC_REJECT,    /**< 参数 CRC 故障注入被拒绝。 */
  A4_NO_POWER_CHECK_NFAULT_FAST_PATH,    /**< nFAULT 快速路径在 1ms 内关闭输出。 */
  A4_NO_POWER_CHECK_WATCHDOG_RESET_SOURCE, /**< 看门狗复位来源可诊断。 */
  A4_NO_POWER_CHECK_COUNT                /**< 验证项数量哨兵。 */
} a4_no_power_check_t;

/** @brief 单项验证结果。 */
typedef enum {
  A4_NO_POWER_RESULT_SKIP = 0, /**< 前置条件不满足或需要人工注入，未判定。 */
  A4_NO_POWER_RESULT_PASS,     /**< 通过。 */
  A4_NO_POWER_RESULT_FAIL      /**< 不通过。 */
} a4_no_power_result_t;

/**
 * @brief 单项验证结果明细。
 *
 * detail 语义按验证项定义：
 * PB12_LOW 为 0 表示低电平；
 * A_OUTPUT_INVALID 为 0 表示输出禁用；
 * B_BRIDGE_HIZ 位 0 表示 PB12 高、位 1 表示 B 路输出使能；
 * PWM_TIMING 为 A 路死区时间，单位 ns；
 * ADC_TRIGGER 位 0 为 A 路触发、位 1 为 B 路触发、位 2 为 B 路同步；
 * SAMPLE_WINDOW 为 A 路采样窗口，单位 ns；
 * CURRENT_CALIBRATION 为两通道零偏偏差，单位 mV；
 * OFFSET_CAPTURE 为零偏极差，单位 LSB；
 * PARAM_CRC_REJECT 为 1 表示非法记录被错误接受；
 * NFAULT_FAST_PATH 为快速路径耗时，单位 ms；
 * WATCHDOG_RESET_SOURCE 为 1 表示已观察到看门狗复位标志。
 */
typedef struct {
  a4_no_power_check_t check;   /**< 验证项编号。 */
  a4_no_power_result_t result; /**< 验证结果。 */
  uint32_t detail;             /**< 结果明细，语义见结构体说明。 */
} a4_no_power_entry_t;

/**
 * @brief 无功率验证报告，全部为静态分配。
 */
typedef struct {
  uint32_t passed;                                          /**< 通过项数。 */
  uint32_t failed;                                          /**< 不通过项数。 */
  uint32_t skipped;                                         /**< 未判定项数。 */
  uint32_t total;                                           /**< 已执行项数。 */
  uint32_t duration_ms;                                     /**< 用例总耗时，单位 ms。 */
  a4_no_power_entry_t entries[A4_NO_POWER_CHECK_COUNT];      /**< 逐项结果，顺序与执行顺序一致。 */
} a4_no_power_report_t;

/** @brief 把验证项编号转换为执行掩码位。 */
#define A4_NO_POWER_CHECK_BIT(check) (UINT32_C(1) << (uint32_t)(check))

/** @brief 阶段 0 闸门相关验证项掩码。 */
#define A4_NO_POWER_MASK_STAGE0                                                          \
  (A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_PB12_LOW) |                                   \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_A_OUTPUT_INVALID) |                           \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_B_BRIDGE_HIZ) |                               \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_PARAM_CRC_REJECT) |                           \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_NFAULT_FAST_PATH) |                           \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_WATCHDOG_RESET_SOURCE))

/** @brief A4 电流采样与 PWM/ADC 触发相关验证项掩码。 */
#define A4_NO_POWER_MASK_CURRENT                                                         \
  (A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_PWM_TIMING) |                                 \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_ADC_TRIGGER) |                                \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_SAMPLE_WINDOW) |                              \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_CURRENT_CALIBRATION) |                        \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_OFFSET_CAPTURE))

/** @brief 全部验证项掩码。 */
#define A4_NO_POWER_MASK_ALL (A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_COUNT) - UINT32_C(1))

/**
 * @brief 故障注入相关验证项掩码。
 *
 * nFAULT 用例会向 AXIS_B 锁存 FAULT_DRV8323_NFAULT，只允许人工调用。
 */
#define A4_NO_POWER_MASK_FAULT_INJECTION                                                 \
  (A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_PARAM_CRC_REJECT) |                          \
   A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_NFAULT_FAST_PATH))

/**
 * @brief 可在自检任务中自动执行的无功率验证项掩码。
 *
 * 覆盖阶段 0 与 A4 电流采样闸门项，排除会锁存故障的 nFAULT 注入项。
 */
#define A4_NO_POWER_MASK_AUTORUN                                                         \
  (A4_NO_POWER_MASK_ALL & ~A4_NO_POWER_CHECK_BIT(A4_NO_POWER_CHECK_NFAULT_FAST_PATH))

/**
 * @brief 执行指定掩码内的无功率验证项并填充报告。
 *
 * @param check_mask 验证项掩码，使用 A4_NO_POWER_CHECK_BIT 组合。
 * @param report 输出报告，必须由调用者静态分配。
 * @return true 表示没有 FAIL 项（可能有 SKIP 项）。
 *
 * 调用上下文：目标板无功率自检任务，必须在 bsp_pwm_init 和 bsp_current_sense_init 之后。
 * 失败行为：报告为空时返回 false；单项前置条件不满足记为 SKIP 并继续后续项。
 */
bool test_a4_no_power_run(uint32_t check_mask, a4_no_power_report_t *report);

/**
 * @brief 执行全部无功率验证项并填充报告。
 *
 * @param report 输出报告，必须由调用者静态分配。
 * @return true 表示没有 FAIL 项（可能有 SKIP 项）。
 *
 * 调用上下文：目标板无功率自检任务。
 * 失败行为：报告为空时返回 false。
 */
bool test_a4_no_power_run_all(a4_no_power_report_t *report);

/**
 * @brief 通过 UART3 非阻塞日志输出报告，供证据记录模板粘贴。
 *
 * @param report 待输出报告。
 * @return 无返回值。
 *
 * 调用上下文：自检任务。
 * 失败行为：报告为空时直接返回；日志口忙时允许丢行，不影响验证结果。
 */
void test_a4_no_power_log_report(const a4_no_power_report_t *report);

/**
 * @brief 返回验证项短名称，用于证据记录。
 *
 * @param check 验证项编号。
 * @return 以零结尾的短名称；编号越界时返回 UNKNOWN。
 *
 * 调用上下文：日志和证据记录。
 * 失败行为：无失败路径。
 */
const char *test_a4_no_power_check_name(a4_no_power_check_t check);

#ifdef __cplusplus
}
#endif

#endif
