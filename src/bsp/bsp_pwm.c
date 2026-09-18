/**
 * @file bsp_pwm.c
 * @brief 双轴 20kHz 中心对齐 PWM、死区和 ADC 注入触发实现。
 *
 * 主要接口：bsp_pwm_init、bsp_pwm_read_status、bsp_pwm_read_registers、
 *          bsp_pwm_outputs_enabled、bsp_pwm_test_enable_outputs。
 * 依赖关系：依赖 AT32F403A 标准外设库、bsp_pin_map.h、bsp_safe_outputs.h 和 bsp_pwm_math 纯计算层。
 * 关键安全约束：初始化末尾调用 bsp_safe_outputs 保证输出禁用；TMR1 的 BRK 输入保持关闭，
 *              因为 PB12 已改作 B 路 INLx 总控；测试波形使能前必须回读确认 PB12 为低。
 */

#include "bsp/bsp_pwm.h"

#include "at32f403a_407.h"
#include "bsp/bsp_pin_map.h"
#include "bsp/bsp_safe_outputs.h"

/** @brief 静态时基状态缓存，禁止动态分配。 */
static bsp_pwm_status_t s_pwm_status[AXIS_COUNT];

/** @brief PWM 模块初始化标志。 */
static bool s_pwm_initialised;

/**
 * @brief 把 A/B 路 PWM 引脚配置为复用输出。
 *
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值，引脚状态由目标板波形和寄存器回读验证。
 */
static void bsp_pwm_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

  /* A 路高边 PA8/PA9/PA10 与低边 PB13/PB14/PB15 使用 TMR1 默认复用位置。 */
  gpio_init_struct.gpio_pins = (uint16_t)(BSP_A_U_HIGH_PIN | BSP_A_V_HIGH_PIN | BSP_A_W_HIGH_PIN);
  gpio_init(BSP_A_U_HIGH_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = (uint16_t)(BSP_A_U_LOW_PIN | BSP_A_V_LOW_PIN | BSP_A_W_LOW_PIN);
  gpio_init(BSP_A_U_LOW_PORT, &gpio_init_struct);

  /* B 路 PB0/PB1 使用 TMR3 默认复用位置，PB7 使用 TMR4 默认复用位置。 */
  gpio_init_struct.gpio_pins = (uint16_t)(BSP_B_U_PIN | BSP_B_V_PIN);
  gpio_init(BSP_B_U_PORT, &gpio_init_struct);

  gpio_init_struct.gpio_pins = (uint16_t)BSP_B_W_PIN;
  gpio_init(BSP_B_W_PORT, &gpio_init_struct);
}

/**
 * @brief 配置一个 PWM 输出通道，初始化阶段保持输出关闭。
 *
 * @param tmr_x 定时器实例。
 * @param channel 通道编号。
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值，通道状态由寄存器回读验证。
 */
static void bsp_pwm_channel_config(tmr_type *tmr_x, tmr_channel_select_type channel)
{
  tmr_output_config_type output_config;

  tmr_output_default_para_init(&output_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output_config.oc_idle_state = FALSE;
  output_config.occ_idle_state = FALSE;
  output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.oc_output_state = FALSE;
  output_config.occ_output_state = FALSE;

  tmr_output_channel_config(tmr_x, channel, &output_config);
  tmr_channel_value_set(tmr_x, channel, (uint16_t)0u);
}

/**
 * @brief 配置定时器中心对齐时基。
 *
 * @param tmr_x 定时器实例。
 * @param prescaler 预分频值。
 * @param period_ticks 自动重装值。
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值。
 */
static void bsp_pwm_counter_config(tmr_type *tmr_x, uint32_t prescaler, uint32_t period_ticks)
{
  tmr_reset(tmr_x);
  tmr_base_init(tmr_x, period_ticks, prescaler);
  tmr_cnt_dir_set(tmr_x, TMR_COUNT_TWO_WAY_1);
  tmr_period_buffer_enable(tmr_x, TRUE);
  tmr_repetition_counter_set(tmr_x, 0u);
}

/**
 * @brief 配置 A 路 TMR1 三相六路输出和死区。
 *
 * @param prescaler 预分频值。
 * @param period_ticks 自动重装值。
 * @param dead_time 死区编码值。
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值；BRK 输入保持关闭以避免 PB12 复用冲突。
 */
static void bsp_pwm_tmr1_config(uint32_t prescaler, uint32_t period_ticks, uint32_t dead_time)
{
  tmr_brkdt_config_type brkdt_config;

  bsp_pwm_counter_config(TMR1, prescaler, period_ticks);

  bsp_pwm_channel_config(TMR1, TMR_SELECT_CHANNEL_1);
  bsp_pwm_channel_config(TMR1, TMR_SELECT_CHANNEL_2);
  bsp_pwm_channel_config(TMR1, TMR_SELECT_CHANNEL_3);

  tmr_brkdt_default_para_init(&brkdt_config);
  brkdt_config.deadtime = (uint8_t)dead_time;
  brkdt_config.brk_enable = FALSE;
  brkdt_config.auto_output_enable = FALSE;
  brkdt_config.fcsoen_state = TRUE;
  brkdt_config.fcsodis_state = TRUE;
  tmr_brkdt_config(TMR1, &brkdt_config);

  /* TRGO 使用更新事件，供从定时器同步和 ADC 注入触发使用。 */
  tmr_primary_mode_select(TMR1, TMR_PRIMARY_SEL_OVERFLOW);
  tmr_output_enable(TMR1, FALSE);
  tmr_counter_enable(TMR1, FALSE);
}

/**
 * @brief 配置 B 路从定时器的时基、输出和同步输入。
 *
 * @param tmr_x 定时器实例。
 * @param prescaler 预分频值。
 * @param period_ticks 自动重装值。
 * @param channels 需要配置的通道掩码位。
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值；从定时器保持停止，等待统一使能。
 */
static void bsp_pwm_subordinate_config(tmr_type *tmr_x,
                                       uint32_t prescaler,
                                       uint32_t period_ticks,
                                       uint32_t channels)
{
  bsp_pwm_counter_config(tmr_x, prescaler, period_ticks);

  if ((channels & 1u) != 0u) {
    bsp_pwm_channel_config(tmr_x, TMR_SELECT_CHANNEL_3);
  }
  if ((channels & 2u) != 0u) {
    bsp_pwm_channel_config(tmr_x, TMR_SELECT_CHANNEL_4);
  }
  if ((channels & 4u) != 0u) {
    bsp_pwm_channel_config(tmr_x, TMR_SELECT_CHANNEL_2);
  }

  /* 从定时器由 TMR1 的 TRGO 同步启动，保证两路载波同源。 */
  tmr_trigger_input_select(tmr_x, TMR_SUB_INPUT_SEL_IS0);
  tmr_sub_mode_select(tmr_x, TMR_SUB_TRIGGER_MODE);
  tmr_sub_sync_mode_set(tmr_x, TRUE);
  tmr_primary_mode_select(tmr_x, TMR_PRIMARY_SEL_OVERFLOW);
  tmr_counter_enable(tmr_x, FALSE);
}

/**
 * @brief 配置 ADC1 与 ADC2 的注入序列和触发源。
 *
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值；触发是否生效由目标板 ADC 波形证据确认。
 */
static void bsp_pwm_adc_trigger_config(void)
{
  adc_base_config_type adc_base_init_struct;
  uint32_t wait_loop;

  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_ADC2_PERIPH_CLOCK, TRUE);

  /* ADC 时钟必须低于器件上限，192MHz 主频下 APB2 分频后再 8 分频。 */
  crm_adc_clock_div_set(CRM_ADC_DIV_8);

  adc_base_default_para_init(&adc_base_init_struct);
  adc_base_init_struct.sequence_mode = FALSE;
  adc_base_init_struct.repeat_mode = FALSE;
  adc_base_init_struct.data_align = ADC_RIGHT_ALIGNMENT;
  adc_base_init_struct.ordinary_channel_length = 1u;

  adc_base_config(ADC1, &adc_base_init_struct);
  adc_base_config(ADC2, &adc_base_init_struct);

  adc_calibration_init(ADC1);
  adc_calibration_init(ADC2);
  for (wait_loop = 0u; wait_loop < 100000u; ++wait_loop) {
    if ((adc_calibration_init_status_get(ADC1) == SET) &&
        (adc_calibration_init_status_get(ADC2) == SET)) {
      break;
    }
  }
  adc_calibration_start(ADC1);
  adc_calibration_start(ADC2);

  /* A 路两相注入序列：PA0 对应 ADC1 通道 0，PA1 对应 ADC1 通道 1。 */
  adc_preempt_channel_length_set(ADC1, 2u);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_0, 1u, ADC_SAMPLETIME_41_5);
  adc_preempt_channel_set(ADC1, ADC_CHANNEL_1, 2u, ADC_SAMPLETIME_41_5);

  /* B 路两相注入序列：PA2 对应 ADC2 通道 2，PA3 对应 ADC2 通道 3。 */
  adc_preempt_channel_length_set(ADC2, 2u);
  adc_preempt_channel_set(ADC2, ADC_CHANNEL_2, 1u, ADC_SAMPLETIME_41_5);
  adc_preempt_channel_set(ADC2, ADC_CHANNEL_3, 2u, ADC_SAMPLETIME_41_5);

  adc_preempt_conversion_trigger_set(ADC1, ADC12_PREEMPT_TRIG_TMR1TRGOUT, TRUE);
  adc_preempt_conversion_trigger_set(ADC2, ADC12_PREEMPT_TRIG_TMR4TRGOUT, TRUE);

  adc_enable(ADC1, TRUE);
  adc_enable(ADC2, TRUE);
}

/**
 * @brief 把模拟输入引脚配置为模拟模式。
 *
 * @return 无返回值。
 *
 * 调用上下文：bsp_pwm_init。
 * 失败行为：无失败返回值。
 */
static void bsp_pwm_analog_gpio_init(void)
{
  gpio_init_type gpio_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  gpio_init_struct.gpio_mode = GPIO_MODE_ANALOG;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_pins = (uint16_t)(BSP_A_IU_PIN | BSP_A_IV_PIN | BSP_B_IU_PIN | BSP_B_IV_PIN);
  gpio_init(BSP_A_IU_PORT, &gpio_init_struct);
}

/**
 * @brief 判断测试波形使能的前置安全条件。
 *
 * @return true 表示 PB12 已确认为低电平。
 *
 * 调用上下文：bsp_pwm_test_enable_outputs。
 * 失败行为：PB12 非低时返回 false，拒绝使能任何输出。
 */
static bool bsp_pwm_test_preconditions_met(void)
{
  return (gpio_output_data_bit_read(BSP_B_INL_PORT, BSP_B_INL_PIN) == 0u);
}

/**
 * @brief 初始化两路 PWM 时基、死区和 ADC 注入触发。
 *
 * @return true 表示时基配置成功且输出保持禁用。
 *
 * 调用上下文：app_init。
 * 失败行为：任一时基参数不可满足时返回 false，并调用 bsp_safe_outputs 保持安全态。
 */
bool bsp_pwm_init(void)
{
  crm_clocks_freq_type clocks;
  uint32_t timer1_clock = 0u;
  uint32_t timer34_clock = 0u;
  uint32_t a_prescaler = 0u;
  uint32_t a_period = 0u;
  uint32_t a_carrier = 0u;
  uint32_t b_prescaler = 0u;
  uint32_t b_period = 0u;
  uint32_t b_carrier = 0u;
  uint32_t dead_time_encoded = 0u;
  uint32_t dead_time_ns = 0u;
  uint32_t min_pulse_ticks = 0u;
  axis_id_t axis;

  if (s_pwm_initialised) {
    return true;
  }

  /* 任何配置动作之前先回到安全态。 */
  bsp_safe_outputs();

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);

  crm_clocks_freq_get(&clocks);
  if (!bsp_pwm_timer_clock_hz(clocks.ahb_freq, clocks.apb2_freq, &timer1_clock) ||
      !bsp_pwm_timer_clock_hz(clocks.ahb_freq, clocks.apb1_freq, &timer34_clock)) {
    bsp_safe_outputs();
    return false;
  }

  if (!bsp_pwm_compute_timing(timer1_clock, BSP_PWM_CARRIER_HZ, &a_prescaler, &a_period, &a_carrier) ||
      !bsp_pwm_compute_timing(timer34_clock, BSP_PWM_CARRIER_HZ, &b_prescaler, &b_period, &b_carrier) ||
      !bsp_pwm_encode_dead_time(timer1_clock, BSP_PWM_DEAD_TIME_MIN_NS, &dead_time_encoded, &dead_time_ns) ||
      !bsp_pwm_min_pulse_ticks(timer1_clock, a_prescaler, BSP_PWM_MIN_PULSE_NS, a_period, &min_pulse_ticks)) {
    bsp_safe_outputs();
    return false;
  }

  bsp_pwm_gpio_init();
  bsp_pwm_analog_gpio_init();

  bsp_pwm_tmr1_config(a_prescaler, a_period, dead_time_encoded);
  bsp_pwm_subordinate_config(TMR3, b_prescaler, b_period, 3u);
  bsp_pwm_subordinate_config(TMR4, b_prescaler, b_period, 4u);
  bsp_pwm_adc_trigger_config();

  for (axis = AXIS_A; axis < AXIS_COUNT; ++axis) {
    s_pwm_status[axis].axis = axis;
    s_pwm_status[axis].center_aligned = true;
    s_pwm_status[axis].counter_running = false;
    s_pwm_status[axis].outputs_enabled = false;
    s_pwm_status[axis].adc_trigger_armed = true;
    s_pwm_status[axis].subordinate_synced = (axis == AXIS_A) ? false : true;
    s_pwm_status[axis].min_pulse_ticks = min_pulse_ticks;

    if (axis == AXIS_A) {
      s_pwm_status[axis].timer_clock_hz = timer1_clock;
      s_pwm_status[axis].carrier_hz = a_carrier;
      s_pwm_status[axis].period_ticks = a_period;
      s_pwm_status[axis].prescaler = a_prescaler;
      s_pwm_status[axis].dead_time_encoded = dead_time_encoded;
      s_pwm_status[axis].dead_time_ns = dead_time_ns;
    } else {
      s_pwm_status[axis].timer_clock_hz = timer34_clock;
      s_pwm_status[axis].carrier_hz = b_carrier;
      s_pwm_status[axis].period_ticks = b_period;
      s_pwm_status[axis].prescaler = b_prescaler;
      s_pwm_status[axis].dead_time_encoded = 0u;
      s_pwm_status[axis].dead_time_ns = dead_time_ns;
    }
  }

  s_pwm_initialised = true;

  /* 使能 TMR1 更新中断作为双轴 20kHz 控制时基。
     优先级(1,0)低于 nFAULT(0,0)，保证故障路径可抢占控制路径。 */
  tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
  nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1u, 0u);

  /* 配置完成后再次强制安全输出，保证无功率默认态。 */
  bsp_safe_outputs();
  return true;
}

/**
 * @brief 读取指定轴的 PWM 时基状态快照。
 *
 * @param axis 轴编号。
 * @param status 输出状态快照。
 * @return true 表示读取成功。
 *
 * 调用上下文：自检、日志和证据记录。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_pwm_read_status(axis_id_t axis, bsp_pwm_status_t *status)
{
  if (!s_pwm_initialised || (status == 0) || (axis >= AXIS_COUNT)) {
    return false;
  }

  *status = s_pwm_status[axis];

  if (axis == AXIS_A) {
    status->counter_running = (TMR1->ctrl1_bit.tmren != 0u);
    status->outputs_enabled = (TMR1->brk_bit.oen != 0u);
  } else {
    status->counter_running = (TMR3->ctrl1_bit.tmren != 0u) || (TMR4->ctrl1_bit.tmren != 0u);
    status->outputs_enabled = (TMR3->ctrl1_bit.tmren != 0u) || (TMR4->ctrl1_bit.tmren != 0u);
  }

  return true;
}

/**
 * @brief 回读指定轴的关键时基寄存器，用于闸门证据。
 *
 * @param axis 轴编号。
 * @param count_dir 输出计数方向寄存器值。
 * @param period 输出自动重装寄存器值。
 * @param prescaler 输出预分频寄存器值。
 * @param dead_time 输出死区寄存器原始值。
 * @return true 表示回读成功。
 *
 * 调用上下文：目标板无功率验证用例。
 * 失败行为：轴号非法或模块未初始化时返回 false。
 */
bool bsp_pwm_read_registers(axis_id_t axis,
                            uint32_t *count_dir,
                            uint32_t *period,
                            uint32_t *prescaler,
                            uint32_t *dead_time)
{
  if (!s_pwm_initialised || (axis >= AXIS_COUNT) ||
      (count_dir == 0) || (period == 0) || (prescaler == 0) || (dead_time == 0)) {
    return false;
  }

  if (axis == AXIS_A) {
    /* cnt_dir 位域位于 ctrl1 的 [6:4]，回读用于确认中心对齐计数方向。 */
    *count_dir = TMR1->ctrl1 & 0x00000070u;
    *period = TMR1->pr;
    *prescaler = TMR1->div;
    *dead_time = TMR1->brk & 0x000000FFu;
  } else {
    /* B 路从定时器同样回读 cnt_dir 位域，确认与主定时器同源。 */
    *count_dir = TMR3->ctrl1 & 0x00000070u;
    *period = TMR3->pr;
    *prescaler = TMR3->div;
    *dead_time = 0u;
  }

  return true;
}

/**
 * @brief 查询指定轴功率输出通道是否使能。
 *
 * @param axis 轴编号。
 * @return true 表示通道已使能。
 *
 * 调用上下文：自检和安全检查。
 * 失败行为：轴号非法时返回 true，保持保守策略。
 */
bool bsp_pwm_outputs_enabled(axis_id_t axis)
{
  if (axis >= AXIS_COUNT) {
    return true;
  }

  if (axis == AXIS_A) {
    return (TMR1->brk_bit.oen != 0u);
  }

  return (TMR3->ctrl1_bit.tmren != 0u) || (TMR4->ctrl1_bit.tmren != 0u);
}

/**
 * @brief 在无功率条件下临时使能或关闭 PWM 测试波形。
 *
 * @param axis 轴编号。
 * @param enable true 表示使能测试波形，false 表示立即回到安全输出。
 * @return true 表示请求被接受。
 *
 * 调用上下文：仅限 A4 无功率验证用例。
 * 失败行为：PB12 未保持低电平、轴号非法或模块未初始化时拒绝使能并返回 false。
 */
bool bsp_pwm_test_enable_outputs(axis_id_t axis, bool enable)
{
  tmr_output_config_type output_config;
  tmr_type *tmr_x;

  if (!s_pwm_initialised || (axis >= AXIS_COUNT)) {
    return false;
  }

  if (!enable) {
    bsp_safe_outputs();
    s_pwm_status[axis].outputs_enabled = false;
    s_pwm_status[axis].counter_running = false;
    return true;
  }

  /* 无功率测试的硬前置条件：B 路 INLx 总控必须保持低电平。 */
  if (!bsp_pwm_test_preconditions_met()) {
    return false;
  }

  tmr_x = (axis == AXIS_A) ? TMR1 : TMR3;

  tmr_output_default_para_init(&output_config);
  output_config.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  output_config.oc_idle_state = FALSE;
  output_config.occ_idle_state = FALSE;
  output_config.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  output_config.oc_output_state = TRUE;
  output_config.occ_output_state = (axis == AXIS_A) ? TRUE : FALSE;

  if (axis == AXIS_A) {
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &output_config);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_2, &output_config);
    tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &output_config);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_output_enable(TMR1, TRUE);
    tmr_counter_enable(TMR1, TRUE);
  } else {
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_3, &output_config);
    tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_4, &output_config);
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_3, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_4, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_counter_enable(TMR3, TRUE);

    tmr_output_channel_config(TMR4, TMR_SELECT_CHANNEL_2, &output_config);
    tmr_channel_value_set(TMR4, TMR_SELECT_CHANNEL_2, (uint16_t)(s_pwm_status[axis].period_ticks / 2u));
    tmr_counter_enable(TMR4, TRUE);
  }

  (void)tmr_x;
  s_pwm_status[axis].outputs_enabled = true;
  s_pwm_status[axis].counter_running = true;
  return true;
}
