/**
 * @file test_a4_no_power.c
 * @brief A4 阶段无功率验证用例与证据记录实现。
 *
 * 主要接口：test_a4_no_power_run、test_a4_no_power_run_all、test_a4_no_power_log_report、
 *          test_a4_no_power_check_name。
 * 依赖关系：依赖 bsp_pwm、bsp_current_sense、bsp_time、bsp_log、bsp_pin_map、motor_current、
 *          safety 和 storage_parameter；不依赖测试框架和动态分配。
 * 关键安全约束：本文件只回读寄存器和本地缓冲区，不写任何功率使能寄存器；
 *              参数 CRC 注入只作用于静态缓冲区，不修改 Flash 内容。
 */

#include "test/test_a4_no_power.h"

#include <stddef.h>

#include "at32f403a_407.h"

#include "bsp/bsp_current_sense.h"
#include "bsp/bsp_log.h"
#include "bsp/bsp_pin_map.h"
#include "bsp/bsp_pwm.h"
#include "bsp/bsp_time.h"
#include "motor/motor_current.h"
#include "safety/safety.h"
#include "storage/storage_parameter.h"

/** @brief 参数 CRC 故障注入使用的静态记录缓冲，长度与 Flash 记录一致。 */
static uint8_t s_parameter_record[STORAGE_PARAMETER_RECORD_SIZE];

/**
 * @brief 把以零结尾的文本追加到日志行缓冲。
 *
 * @param buffer 目标缓冲。
 * @param index 当前写入位置。
 * @param capacity 缓冲容量，单位 byte。
 * @param text 待追加文本。
 * @return 追加后的写入位置。
 *
 * 调用上下文：证据日志拼装。
 * 失败行为：缓冲写满时停止追加，不越界。
 */
static uint32_t a4_append_text(char *buffer, uint32_t index, uint32_t capacity, const char *text)
{
  while ((*text != '\0') && (index < capacity)) {
    buffer[index] = *text;
    ++index;
    ++text;
  }

  return index;
}

/**
 * @brief 把无符号整数按十进制追加到日志行缓冲。
 *
 * @param buffer 目标缓冲。
 * @param index 当前写入位置。
 * @param capacity 缓冲容量，单位 byte。
 * @param value 待追加数值。
 * @return 追加后的写入位置。
 *
 * 调用上下文：证据日志拼装。
 * 失败行为：数值超过 10 位或缓冲写满时截断，不越界。
 */
static uint32_t a4_append_u32(char *buffer, uint32_t index, uint32_t capacity, uint32_t value)
{
  char digits[10];
  uint32_t count = 0u;

  if (value == 0u) {
    digits[count] = '0';
    ++count;
  }

  while ((value != 0u) && (count < (uint32_t)sizeof(digits))) {
    digits[count] = (char)('0' + (char)(value % 10u));
    ++count;
    value /= 10u;
  }

  while ((count != 0u) && (index < capacity)) {
    --count;
    buffer[index] = digits[count];
    ++index;
  }

  return index;
}

/**
 * @brief 返回验证结果短名称。
 *
 * @param result 验证结果。
 * @return 以零结尾的短名称。
 *
 * 调用上下文：证据日志拼装。
 * 失败行为：取值越界时返回 UNKNOWN。
 */
static const char *a4_result_name(a4_no_power_result_t result)
{
  switch (result) {
    case A4_NO_POWER_RESULT_PASS:
      return "PASS";
    case A4_NO_POWER_RESULT_FAIL:
      return "FAIL";
    case A4_NO_POWER_RESULT_SKIP:
      return "SKIP";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 校验 PB12 是否保持低电平。
 *
 * @param detail 输出结果明细。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PB12 为高电平时返回 FAIL。
 */
static a4_no_power_result_t a4_check_pb12_low(uint32_t *detail)
{
  if (gpio_output_data_bit_read(BSP_B_INL_PORT, BSP_B_INL_PIN) == SET) {
    *detail = UINT32_C(1);
    return A4_NO_POWER_RESULT_FAIL;
  }

  *detail = UINT32_C(0);
  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 校验 A 路功率输出通道处于禁用状态。
 *
 * @param detail 输出结果明细。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PWM 时基未初始化时返回 SKIP，输出使能时返回 FAIL。
 */
static a4_no_power_result_t a4_check_a_output_invalid(uint32_t *detail)
{
  bsp_pwm_status_t status;

  if (!bsp_pwm_read_status(AXIS_A, &status)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  if (bsp_pwm_outputs_enabled(AXIS_A) || status.outputs_enabled) {
    *detail = UINT32_C(1);
    return A4_NO_POWER_RESULT_FAIL;
  }

  *detail = UINT32_C(0);
  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 校验 B 路三相桥臂处于 Hi-Z 条件。
 *
 * @param detail 输出结果明细。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PB12 为高或 B 路输出使能时返回 FAIL，明细按位标记。
 */
static a4_no_power_result_t a4_check_b_bridge_hiz(uint32_t *detail)
{
  uint32_t flags = 0u;

  if (gpio_output_data_bit_read(BSP_B_INL_PORT, BSP_B_INL_PIN) == SET) {
    flags |= UINT32_C(1);
  }

  if (bsp_pwm_outputs_enabled(AXIS_B)) {
    flags |= UINT32_C(2);
  }

  *detail = flags;
  return (flags == 0u) ? A4_NO_POWER_RESULT_PASS : A4_NO_POWER_RESULT_FAIL;
}

/**
 * @brief 校验 20kHz 中心对齐时基、死区和最小脉宽。
 *
 * @param detail 输出 A 路死区时间，单位 ns。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PWM 时基未初始化时返回 SKIP，任何门槛不满足时返回 FAIL。
 */
static a4_no_power_result_t a4_check_pwm_timing(uint32_t *detail)
{
  bsp_pwm_status_t status_a;
  bsp_pwm_status_t status_b;
  uint32_t count_dir = 0u;
  uint32_t period = 0u;
  uint32_t prescaler = 0u;
  uint32_t dead_time = 0u;

  if (!bsp_pwm_read_status(AXIS_A, &status_a) || !bsp_pwm_read_status(AXIS_B, &status_b)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  *detail = status_a.dead_time_ns;

  if (!bsp_pwm_read_registers(AXIS_A, &count_dir, &period, &prescaler, &dead_time)) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (!bsp_pwm_timing_is_within_limits(&status_a) || !bsp_pwm_timing_is_within_limits(&status_b)) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (status_a.carrier_hz != status_b.carrier_hz) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (!status_b.subordinate_synced) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (count_dir == 0u) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if ((period != status_a.period_ticks) || (prescaler != status_a.prescaler) ||
      (dead_time != status_a.dead_time_encoded)) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 校验 ADC 注入触发与 TMR3/TMR4 同步已装订。
 *
 * @param detail 输出标志位，位 0 为 A 路触发、位 1 为 B 路触发、位 2 为 B 路同步。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PWM 时基未初始化时返回 SKIP，任一触发未配置时返回 FAIL。
 */
static a4_no_power_result_t a4_check_adc_trigger(uint32_t *detail)
{
  bsp_pwm_status_t status_a;
  bsp_pwm_status_t status_b;
  uint32_t flags = 0u;

  if (!bsp_pwm_read_status(AXIS_A, &status_a) || !bsp_pwm_read_status(AXIS_B, &status_b)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  if (status_a.adc_trigger_armed) {
    flags |= UINT32_C(1);
  }
  if (status_b.adc_trigger_armed) {
    flags |= UINT32_C(2);
  }
  if (status_b.subordinate_synced) {
    flags |= UINT32_C(4);
  }

  *detail = flags;

  if ((flags != UINT32_C(7)) || (status_a.timer_clock_hz == 0u) || (status_b.timer_clock_hz == 0u)) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 校验注入采样窗口不低于下限。
 *
 * @param detail 输出 A 路采样窗口，单位 ns。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：PWM 或采样模块未初始化时返回 SKIP，窗口不足时返回 FAIL。
 */
static a4_no_power_result_t a4_check_sample_window(uint32_t *detail)
{
  bsp_pwm_status_t status_a;
  bsp_pwm_status_t status_b;
  bsp_current_sense_status_t sense;
  bool window_ok;

  if (!bsp_pwm_read_status(AXIS_A, &status_a) || !bsp_pwm_read_status(AXIS_B, &status_b)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  window_ok = bsp_current_sense_evaluate_window(AXIS_A, status_a.carrier_hz, status_a.dead_time_ns);
  window_ok = bsp_current_sense_evaluate_window(AXIS_B, status_b.carrier_hz, status_b.dead_time_ns) &&
              window_ok;

  if (!bsp_current_sense_read_status(AXIS_A, &sense)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  *detail = sense.sample_window_ns;
  return window_ok ? A4_NO_POWER_RESULT_PASS : A4_NO_POWER_RESULT_FAIL;
}

/**
 * @brief 校验电流量程、增益误差和零偏误差门槛。
 *
 * @param detail 输出两通道零偏偏差，单位 mV。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：默认标定生成失败或任一门槛不满足时返回 FAIL。
 */
static a4_no_power_result_t a4_check_current_calibration(uint32_t *detail)
{
  motor_current_calibration_t calibration;
  float gain_error_ratio = 0.0f;
  uint32_t offset_error_mv = 0u;

  if (!motor_current_default_calibration(&calibration)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (!motor_current_validate_calibration(&calibration, &gain_error_ratio, &offset_error_mv)) {
    *detail = offset_error_mv;
    return A4_NO_POWER_RESULT_FAIL;
  }

  *detail = offset_error_mv;

  if (calibration.range_a < MOTOR_CURRENT_RANGE_MIN_A) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 在零电流条件下采集两相零偏并检查离散度。
 *
 * @param detail 输出零偏极差，单位 LSB。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例，必须在功率输出禁用条件下执行。
 * 失败行为：采样模块未初始化时返回 SKIP，极差超限时返回 FAIL。
 */
static a4_no_power_result_t a4_check_offset_capture(uint32_t *detail)
{
  bsp_current_sense_status_t sense;
  bool capture_a;
  bool capture_b;

  if (!bsp_current_sense_read_status(AXIS_A, &sense)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  capture_a = bsp_current_sense_capture_offsets(AXIS_A);
  capture_b = bsp_current_sense_capture_offsets(AXIS_B);

  if (!bsp_current_sense_read_status(AXIS_A, &sense)) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  *detail = sense.offset_spread_lsb;

  if (!capture_a || !capture_b) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (!bsp_current_sense_offsets_valid(AXIS_A) || !bsp_current_sense_offsets_valid(AXIS_B)) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 注入参数记录 CRC 错误并确认解析被拒绝。
 *
 * @param detail 输出 1 表示非法记录被错误接受。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例，只操作静态缓冲区。
 * 失败行为：合法记录无法构建或解析，或者损坏记录被接受时返回 FAIL。
 */
static a4_no_power_result_t a4_check_parameter_crc_reject(uint32_t *detail)
{
  parameter_set_t source;
  parameter_set_t parsed;
  uint32_t sequence = 0u;

  storage_parameter_defaults(&source);

  if (!storage_parameter_build_record(&source, 1u, s_parameter_record,
                                      (uint32_t)sizeof(s_parameter_record))) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (!storage_parameter_parse_record(s_parameter_record, (uint32_t)sizeof(s_parameter_record),
                                      &parsed, &sequence)) {
    *detail = UINT32_C(1);
    return A4_NO_POWER_RESULT_FAIL;
  }

  /* 破坏正文首字节，模拟 Flash 位翻转造成的 CRC 失配。 */
  s_parameter_record[STORAGE_PARAMETER_HEADER_SIZE] ^= 0xFFu;

  if (storage_parameter_parse_record(s_parameter_record, (uint32_t)sizeof(s_parameter_record),
                                     &parsed, &sequence)) {
    *detail = UINT32_C(1);
    return A4_NO_POWER_RESULT_FAIL;
  }

  *detail = UINT32_C(0);
  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 触发 nFAULT 快速路径并检查输出关闭与故障锁存。
 *
 * @param detail 输出快速路径耗时，单位 ms。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例，必须最后执行。
 * 失败行为：时基未运行时返回 SKIP；输出未关闭、故障未锁存或耗时超限时返回 FAIL。
 */
static a4_no_power_result_t a4_check_nfault_fast_path(uint32_t *detail)
{
  uint32_t start_ms;
  uint32_t elapsed_ms;
  bool outputs_off;
  bool fault_latched;

  start_ms = bsp_time_get_ms();
  if (start_ms == 0u) {
    *detail = UINT32_C(0);
    return A4_NO_POWER_RESULT_SKIP;
  }

  safety_fast_fault((uint32_t)FAULT_DRV8323_NFAULT, 0);
  elapsed_ms = bsp_time_get_ms() - start_ms;

  outputs_off = !bsp_pwm_outputs_enabled(AXIS_A) && !bsp_pwm_outputs_enabled(AXIS_B);
  fault_latched = (safety_get_fault_flags(AXIS_B) & (uint32_t)FAULT_DRV8323_NFAULT) != 0u;

  *detail = elapsed_ms;

  if (!outputs_off || !fault_latched) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  if (elapsed_ms > A4_NO_POWER_NFAULT_DEADLINE_MS) {
    return A4_NO_POWER_RESULT_FAIL;
  }

  return A4_NO_POWER_RESULT_PASS;
}

/**
 * @brief 读取看门狗复位来源标志。
 *
 * @param detail 输出 1 表示已观察到看门狗复位标志。
 * @return 验证结果。
 *
 * 调用上下文：A4 无功率验证用例。
 * 失败行为：未观察到标志时返回 SKIP，需要人工注入看门狗复位后重新上电复测。
 */
static a4_no_power_result_t a4_check_watchdog_reset_source(uint32_t *detail)
{
  if (crm_flag_get(CRM_WDT_RESET_FLAG) == SET) {
    *detail = UINT32_C(1);
    return A4_NO_POWER_RESULT_PASS;
  }

  *detail = UINT32_C(0);
  return A4_NO_POWER_RESULT_SKIP;
}

/**
 * @brief 按编号分发单项验证。
 *
 * @param check 验证项编号。
 * @param detail 输出结果明细。
 * @return 验证结果。
 *
 * 调用上下文：test_a4_no_power_run 内部。
 * 失败行为：编号越界时返回 SKIP。
 */
static a4_no_power_result_t a4_run_check(a4_no_power_check_t check, uint32_t *detail)
{
  switch (check) {
    case A4_NO_POWER_CHECK_PB12_LOW:
      return a4_check_pb12_low(detail);
    case A4_NO_POWER_CHECK_A_OUTPUT_INVALID:
      return a4_check_a_output_invalid(detail);
    case A4_NO_POWER_CHECK_B_BRIDGE_HIZ:
      return a4_check_b_bridge_hiz(detail);
    case A4_NO_POWER_CHECK_PWM_TIMING:
      return a4_check_pwm_timing(detail);
    case A4_NO_POWER_CHECK_ADC_TRIGGER:
      return a4_check_adc_trigger(detail);
    case A4_NO_POWER_CHECK_SAMPLE_WINDOW:
      return a4_check_sample_window(detail);
    case A4_NO_POWER_CHECK_CURRENT_CALIBRATION:
      return a4_check_current_calibration(detail);
    case A4_NO_POWER_CHECK_OFFSET_CAPTURE:
      return a4_check_offset_capture(detail);
    case A4_NO_POWER_CHECK_PARAM_CRC_REJECT:
      return a4_check_parameter_crc_reject(detail);
    case A4_NO_POWER_CHECK_NFAULT_FAST_PATH:
      return a4_check_nfault_fast_path(detail);
    case A4_NO_POWER_CHECK_WATCHDOG_RESET_SOURCE:
      return a4_check_watchdog_reset_source(detail);
    case A4_NO_POWER_CHECK_COUNT:
    default:
      *detail = UINT32_C(0);
      return A4_NO_POWER_RESULT_SKIP;
  }
}

/**
 * @brief 把单项结果写入报告并统计。
 *
 * @param report 输出报告。
 * @param check 验证项编号。
 * @param result 验证结果。
 * @param detail 结果明细。
 * @return 无返回值。
 *
 * 调用上下文：test_a4_no_power_run 内部。
 * 失败行为：报告条目已满时丢弃该项，不影响已完成统计。
 */
static void a4_record_result(a4_no_power_report_t *report,
                             a4_no_power_check_t check,
                             a4_no_power_result_t result,
                             uint32_t detail)
{
  a4_no_power_entry_t *entry;

  if (report->total >= (uint32_t)A4_NO_POWER_CHECK_COUNT) {
    return;
  }

  entry = &report->entries[report->total];
  entry->check = check;
  entry->result = result;
  entry->detail = detail;
  report->total++;

  if (result == A4_NO_POWER_RESULT_PASS) {
    report->passed++;
  } else if (result == A4_NO_POWER_RESULT_FAIL) {
    report->failed++;
  } else {
    report->skipped++;
  }
}

/**
 * @brief 输出单项证据日志行。
 *
 * @param check 验证项编号。
 * @param result 验证结果。
 * @param detail 结果明细。
 * @return 无返回值。
 *
 * 调用上下文：test_a4_no_power_run 内部。
 * 失败行为：日志口忙时丢行，不影响验证结果。
 */
static void a4_log_entry(a4_no_power_check_t check, a4_no_power_result_t result, uint32_t detail)
{
  static char line[A4_NO_POWER_LOG_LINE_SIZE];
  uint32_t index = 0u;

  index = a4_append_text(line, index, (uint32_t)sizeof(line), "A4-NP ");
  index = a4_append_text(line, index, (uint32_t)sizeof(line), test_a4_no_power_check_name(check));
  index = a4_append_text(line, index, (uint32_t)sizeof(line), " -> ");
  index = a4_append_text(line, index, (uint32_t)sizeof(line), a4_result_name(result));
  index = a4_append_text(line, index, (uint32_t)sizeof(line), " detail=");
  index = a4_append_u32(line, index, (uint32_t)sizeof(line), detail);
  index = a4_append_text(line, index, (uint32_t)sizeof(line), "\r\n");

  (void)bsp_log_write_nonblocking(line, index);
}

/**
 * @brief 返回验证项短名称。
 *
 * @param check 验证项编号。
 * @return 以零结尾的短名称。
 *
 * 调用上下文：日志和证据记录。
 * 失败行为：编号越界时返回 UNKNOWN。
 */
const char *test_a4_no_power_check_name(a4_no_power_check_t check)
{
  switch (check) {
    case A4_NO_POWER_CHECK_PB12_LOW:
      return "PB12_LOW";
    case A4_NO_POWER_CHECK_A_OUTPUT_INVALID:
      return "A_OUTPUT_INVALID";
    case A4_NO_POWER_CHECK_B_BRIDGE_HIZ:
      return "B_BRIDGE_HIZ";
    case A4_NO_POWER_CHECK_PWM_TIMING:
      return "PWM_TIMING";
    case A4_NO_POWER_CHECK_ADC_TRIGGER:
      return "ADC_TRIGGER";
    case A4_NO_POWER_CHECK_SAMPLE_WINDOW:
      return "SAMPLE_WINDOW";
    case A4_NO_POWER_CHECK_CURRENT_CALIBRATION:
      return "CURRENT_CALIBRATION";
    case A4_NO_POWER_CHECK_OFFSET_CAPTURE:
      return "OFFSET_CAPTURE";
    case A4_NO_POWER_CHECK_PARAM_CRC_REJECT:
      return "PARAM_CRC_REJECT";
    case A4_NO_POWER_CHECK_NFAULT_FAST_PATH:
      return "NFAULT_FAST_PATH";
    case A4_NO_POWER_CHECK_WATCHDOG_RESET_SOURCE:
      return "WATCHDOG_RESET_SOURCE";
    case A4_NO_POWER_CHECK_COUNT:
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief 执行指定掩码内的无功率验证项并填充报告。
 *
 * @param check_mask 验证项掩码。
 * @param report 输出报告。
 * @return true 表示没有 FAIL 项。
 *
 * 调用上下文：目标板无功率自检任务。
 * 失败行为：报告为空时返回 false；单项失败不中断后续项。
 */
bool test_a4_no_power_run(uint32_t check_mask, a4_no_power_report_t *report)
{
  uint32_t start_ms;
  a4_no_power_check_t check;

  if (report == 0) {
    return false;
  }

  report->passed = 0u;
  report->failed = 0u;
  report->skipped = 0u;
  report->total = 0u;
  report->duration_ms = 0u;

  start_ms = bsp_time_get_ms();

  for (check = A4_NO_POWER_CHECK_PB12_LOW; check < A4_NO_POWER_CHECK_COUNT; ++check) {
    uint32_t detail = 0u;
    a4_no_power_result_t result;

    if ((check_mask & A4_NO_POWER_CHECK_BIT(check)) == 0u) {
      continue;
    }

    result = a4_run_check(check, &detail);
    a4_record_result(report, check, result, detail);
    a4_log_entry(check, result, detail);
  }

  report->duration_ms = bsp_time_get_ms() - start_ms;
  return (report->failed == 0u);
}

/**
 * @brief 执行全部无功率验证项并填充报告。
 *
 * @param report 输出报告。
 * @return true 表示没有 FAIL 项。
 *
 * 调用上下文：目标板无功率自检任务。
 * 失败行为：报告为空时返回 false。
 */
bool test_a4_no_power_run_all(a4_no_power_report_t *report)
{
  return test_a4_no_power_run(A4_NO_POWER_MASK_ALL, report);
}

/**
 * @brief 通过 UART3 非阻塞日志输出报告。
 *
 * @param report 待输出报告。
 * @return 无返回值。
 *
 * 调用上下文：自检任务。
 * 失败行为：报告为空时直接返回；日志口忙时丢行。
 */
void test_a4_no_power_log_report(const a4_no_power_report_t *report)
{
  static char line[A4_NO_POWER_LOG_LINE_SIZE];
  uint32_t index = 0u;
  uint32_t entry_index;

  if (report == 0) {
    return;
  }

  index = a4_append_text(line, index, (uint32_t)sizeof(line), "A4-NP summary pass=");
  index = a4_append_u32(line, index, (uint32_t)sizeof(line), report->passed);
  index = a4_append_text(line, index, (uint32_t)sizeof(line), " fail=");
  index = a4_append_u32(line, index, (uint32_t)sizeof(line), report->failed);
  index = a4_append_text(line, index, (uint32_t)sizeof(line), " skip=");
  index = a4_append_u32(line, index, (uint32_t)sizeof(line), report->skipped);
  index = a4_append_text(line, index, (uint32_t)sizeof(line), " duration_ms=");
  index = a4_append_u32(line, index, (uint32_t)sizeof(line), report->duration_ms);
  index = a4_append_text(line, index, (uint32_t)sizeof(line), "\r\n");
  (void)bsp_log_write_nonblocking(line, index);

  for (entry_index = 0u; entry_index < report->total; ++entry_index) {
    a4_log_entry(report->entries[entry_index].check,
                 report->entries[entry_index].result,
                 report->entries[entry_index].detail);
  }
}
