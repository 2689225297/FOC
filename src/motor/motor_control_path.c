/**
 * @file motor_control_path.c
 * @brief A5 单轴控制路径编排实现（启动策略 + 观测器仲裁 + 电流环 + 安全停机）。
 *
 * 本文件把三个产品级算法模块编排成一条可测试的单轴控制路径：
 *   Clarke → 观测器仲裁 → 启动策略 → 角度源选择 → （V/F 电压通路 | 电流环通路）
 *   → SVPWM 占空比 → 故障与安全停机判定。
 *
 * 电压反馈使用"上一拍下发电压"（v_ab），与产品实现中 ADC 采样相对 PWM 下发的
 * 一拍延迟一致，因此观测器输入无需额外延迟补偿。
 *
 * 本文件为纯数值实现，不访问寄存器、不分配内存、不调用 BSP。
 */

#include "motor/motor_control_path.h"

#include <math.h>
#include <stddef.h>

#include "motor/motor_math.h"
#include "motor/motor_transform.h"

/** @brief 安全停机时下发的零电压中点占空比。 */
#define MOTOR_CONTROL_PATH_NEUTRAL_DUTY 0.5f

/** @brief 电角速度到机械转速的换算系数。 */
#define MOTOR_CONTROL_PATH_RAD_S_TO_RPM 9.549296585513721f

/** @brief 单轴控制路径运行上下文。 */
typedef struct
{
  motor_control_state_t control;      /**< 电流环状态。 */
  motor_startup_state_t startup;      /**< 启动策略状态。 */
  motor_observer_state_t observer;    /**< 观测器仲裁状态。 */
  motor_dq_t idq_ref_cmd;             /**< 闭环电流指令（交班时锁存）。 */
  motor_control_path_status_t status; /**< 最近一拍状态。 */
  bool initialized;                   /**< 上下文是否已构造。 */
  bool active;                        /**< 是否处于运行态。 */
  bool closed_loop_latched;           /**< 是否已锁存闭环交班。 */
  bool stop_requested;                /**< 是否已请求安全停机。 */
} motor_control_path_context_t;

/** @brief 每轴独立静态上下文，不使用动态内存。 */
static motor_control_path_context_t g_control_paths[MOTOR_AXIS_COUNT];

/**
 * @brief 判断轴编号是否合法。
 *
 * @param axis 轴编号。
 * @return 合法返回 true。
 */
static bool control_path_axis_valid(axis_id_t axis)
{
  return ((uint32_t)axis < (uint32_t)MOTOR_AXIS_COUNT);
}

/**
 * @brief 把上下文置为安全停机态。
 *
 * @param ctx 控制路径上下文。
 * @return 无返回值。
 */
static void control_path_force_safe_stop(motor_control_path_context_t *ctx)
{
  uint32_t index;

  (void)motor_control_safe_stop(&ctx->control);
  ctx->stop_requested = true;
  ctx->active = false;
  ctx->status.stopped = true;
  for (index = 0u; index < 3u; ++index)
  {
    ctx->status.duty[index] = MOTOR_CONTROL_PATH_NEUTRAL_DUTY;
  }
}

/**
 * @brief 填充控制路径默认配置。
 *
 * @param config 待填充配置。
 * @return 指针非空返回 true。
 */
bool motor_control_path_default_config(motor_control_path_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }

  (void)motor_control_default_config(&config->control);
  (void)motor_startup_default_config(&config->startup);
  (void)motor_observer_default_config(&config->observer);

  return true;
}

/**
 * @brief 校验控制路径配置合法性。
 *
 * @param config 待校验配置。
 * @return 合法返回 true。
 */
bool motor_control_path_validate_config(const motor_control_path_config_t *config)
{
  if (config == NULL)
  {
    return false;
  }
  if (!motor_control_validate_config(&config->control))
  {
    return false;
  }
  if (!motor_startup_validate_config(&config->startup))
  {
    return false;
  }
  if (!motor_observer_validate_config(&config->observer))
  {
    return false;
  }
  if (!motor_is_finite(config->observer.pole_pairs) || (config->observer.pole_pairs <= 0.0f))
  {
    return false;
  }

  return true;
}

/**
 * @brief 构造指定轴控制路径（不进入运行态）。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @return 成功返回 true。
 */
bool motor_control_path_init(axis_id_t axis, const motor_control_path_config_t *config)
{
  motor_control_path_context_t *ctx;

  if (!control_path_axis_valid(axis) || !motor_control_path_validate_config(config))
  {
    return false;
  }

  ctx = &g_control_paths[(uint32_t)axis];
  if (ctx->active)
  {
    /* 运行态禁止替换控制路径配置。 */
    return false;
  }

  if (!motor_control_init(&ctx->control, &config->control))
  {
    return false;
  }
  if (!motor_observer_reset(&ctx->observer, &config->observer, 0.0f, 0.0f))
  {
    return false;
  }
  if (!motor_startup_reset(&ctx->startup, &config->startup))
  {
    return false;
  }

  ctx->idq_ref_cmd.d = 0.0f;
  ctx->idq_ref_cmd.q = 0.0f;
  ctx->initialized = true;
  ctx->active = false;
  ctx->closed_loop_latched = false;
  ctx->stop_requested = false;
  ctx->status.phase = ctx->startup.phase;
  ctx->status.observer = MOTOR_OBSERVER_ACTIVE_NONE;
  ctx->status.mode = MOTOR_MODE_STOP;
  ctx->status.duty[0] = MOTOR_CONTROL_PATH_NEUTRAL_DUTY;
  ctx->status.duty[1] = MOTOR_CONTROL_PATH_NEUTRAL_DUTY;
  ctx->status.duty[2] = MOTOR_CONTROL_PATH_NEUTRAL_DUTY;
  ctx->status.modulation_index = 0.0f;
  ctx->status.theta_used_rad = 0.0f;
  ctx->status.observer_speed_rpm = 0.0f;
  ctx->status.smo_speed_rpm = 0.0f;
  ctx->status.v_limit_v = ctx->control.v_limit_v;
  ctx->status.hfi_error = 0.0f;
  ctx->status.angle_error_rad = 0.0f;
  ctx->status.fault_flags = 0u;
  ctx->status.limit_count = 0u;
  ctx->status.fallback_count = 0u;
  ctx->status.closed_loop = false;
  ctx->status.start_failed = false;
  ctx->status.observer_diverged = false;
  ctx->status.flux_monitor_ok = false;
  ctx->status.limited = false;
  ctx->status.duty_safe = true;
  ctx->status.stopped = true;

  return true;
}

/**
 * @brief 启动指定轴控制路径。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @return 成功返回 true。
 */
bool motor_control_path_start(axis_id_t axis, const motor_control_path_config_t *config)
{
  motor_control_path_context_t *ctx;

  if (!control_path_axis_valid(axis) || !motor_control_path_validate_config(config))
  {
    return false;
  }

  ctx = &g_control_paths[(uint32_t)axis];
  if (!ctx->initialized)
  {
    if (!motor_control_path_init(axis, config))
    {
      return false;
    }
  }

  /* 每次启动都必须复位三个子模块，保证积分、锁相与相位对齐从零开始。 */
  if (!motor_control_init(&ctx->control, &config->control))
  {
    return false;
  }
  if (!motor_startup_reset(&ctx->startup, &config->startup))
  {
    return false;
  }
  if (!motor_observer_reset(&ctx->observer,
                           &config->observer,
                           ctx->startup.if_state.theta_e_rad,
                           ctx->startup.if_state.omega_e_rad_s))
  {
    return false;
  }

  ctx->idq_ref_cmd.d = 0.0f;
  ctx->idq_ref_cmd.q = 0.0f;
  ctx->closed_loop_latched = false;
  ctx->stop_requested = false;
  ctx->active = true;
  ctx->status.fault_flags = 0u;
  ctx->status.closed_loop = false;
  ctx->status.start_failed = false;
  ctx->status.observer_diverged = false;
  ctx->status.stopped = false;
  ctx->status.phase = ctx->startup.phase;
  ctx->status.limit_count = 0u;
  ctx->status.fallback_count = 0u;

  return true;
}

/**
 * @brief 立即进入安全停机并锁存停机请求。
 *
 * @param axis 轴编号。
 * @return true 表示已进入安全停机。
 */
bool motor_control_path_safe_stop(axis_id_t axis)
{
  if (!control_path_axis_valid(axis))
  {
    return false;
  }

  control_path_force_safe_stop(&g_control_paths[(uint32_t)axis]);

  return true;
}

/**
 * @brief 查询控制路径是否处于运行态。
 *
 * @param axis 轴编号。
 * @return true 表示已启动且未安全停机。
 */
bool motor_control_path_is_active(axis_id_t axis)
{
  if (!control_path_axis_valid(axis))
  {
    return false;
  }

  return g_control_paths[(uint32_t)axis].active;
}

/**
 * @brief 读取控制路径状态副本。
 *
 * @param axis 轴编号。
 * @param status 输出状态。
 * @return 成功返回 true。
 */
bool motor_control_path_get_status(axis_id_t axis, motor_control_path_status_t *status)
{
  motor_control_path_context_t *ctx;

  if (!control_path_axis_valid(axis) || (status == NULL))
  {
    return false;
  }

  ctx = &g_control_paths[(uint32_t)axis];
  if (!ctx->initialized)
  {
    return false;
  }

  *status = ctx->status;

  return true;
}

/**
 * @brief 推进一个控制周期的单轴控制路径。
 *
 * @param axis 轴编号。
 * @param config 控制路径配置。
 * @param i_abc 三相实测电流 A。
 * @param dt_s 控制周期 s。
 * @return 成功返回 true。
 */
bool motor_control_path_step(axis_id_t axis,
                            const motor_control_path_config_t *config,
                            const motor_abc_t *i_abc,
                            float dt_s)
{
  motor_control_path_context_t *ctx;
  motor_alpha_beta_t i_ab;
  motor_startup_output_t startup_out;
  float iq_sample;
  float theta_used;
  float omega_used;
  float mech_speed_rpm;
  bool observer_ready;
  bool observer_valid;
  bool allowed;
  bool step_ok;

  if (!control_path_axis_valid(axis) || !motor_control_path_validate_config(config) ||
      (i_abc == NULL))
  {
    return false;
  }
  if (!motor_is_finite(dt_s) || (dt_s <= 0.0f))
  {
    return false;
  }
  if (!motor_is_finite(i_abc->a) || !motor_is_finite(i_abc->b) || !motor_is_finite(i_abc->c))
  {
    return false;
  }

  ctx = &g_control_paths[(uint32_t)axis];
  if (!ctx->initialized || !ctx->active || ctx->stop_requested)
  {
    /* 未构造、未启动或已安全停机时拒绝输出，必须重新启动才能恢复。 */
    return false;
  }

  if (!motor_clarke(i_abc, &i_ab))
  {
    ctx->status.fault_flags = (uint32_t)FAULT_INTERNAL;
    control_path_force_safe_stop(ctx);
    return false;
  }

  /* 观测器使用上一拍下发电压，与 ADC 采样相对 PWM 下发的一拍延迟一致。 */
  if (!motor_observer_step(&ctx->observer, &config->observer, &ctx->control.v_ab, &i_ab, dt_s))
  {
    ctx->status.fault_flags = (uint32_t)FAULT_INTERNAL;
    control_path_force_safe_stop(ctx);
    return false;
  }

  iq_sample = ctx->control.idq.q;
  observer_ready = motor_observer_can_close_loop(&ctx->observer, &config->observer);
  observer_valid = motor_observer_is_angle_valid(&ctx->observer);

  if (!motor_startup_step(&ctx->startup,
                         &config->startup,
                         iq_sample,
                         observer_ready,
                         observer_valid,
                         dt_s,
                         &startup_out))
  {
    ctx->status.fault_flags = (uint32_t)FAULT_INTERNAL;
    control_path_force_safe_stop(ctx);
    return false;
  }

  if (startup_out.closed_loop && !ctx->closed_loop_latched)
  {
    /* 交班瞬间锁存闭环电流指令：I/F 沿用强制电流避免跳变，V/F 以零电流起步。 */
    ctx->closed_loop_latched = true;
    if (config->startup.use_vf_mode)
    {
      ctx->idq_ref_cmd.d = 0.0f;
      ctx->idq_ref_cmd.q = 0.0f;
    }
    else
    {
      ctx->idq_ref_cmd = ctx->control.idq_ref;
    }
  }

  if (startup_out.closed_loop)
  {
    theta_used = ctx->observer.theta_rad;
    omega_used = ctx->observer.omega_e_rad_s;
    mech_speed_rpm = ctx->observer.speed_rpm;
    allowed = observer_valid;
  }
  else
  {
    /* 开环阶段使用强拖角，观测器只作监测与交叉校验。 */
    theta_used = startup_out.theta_rad;
    omega_used = startup_out.omega_e_rad_s;
    mech_speed_rpm = ctx->observer.speed_rpm;
    allowed = true;
  }

  if (!allowed)
  {
    /* 闭环期间观测器角度失效：本拍拒绝输出，由启动策略按超时门限决定回退。 */
    ctx->status.fault_flags = (uint32_t)FAULT_OBSERVER_DIVERGED;
    control_path_force_safe_stop(ctx);
    return false;
  }

  /* HFI 注入电压必须真正下发：作为 d 轴叠加项交给电流环，与 PI 输出一起限幅。 */
  if (!motor_control_set_voltage_injection(&ctx->control, startup_out.hfi_inject_voltage_v))
  {
    ctx->status.fault_flags = (uint32_t)FAULT_INTERNAL;
    control_path_force_safe_stop(ctx);
    return false;
  }

  step_ok = motor_control_set_current_reference(&ctx->control,
                                                &config->control,
                                                startup_out.closed_loop ? &ctx->idq_ref_cmd
                                                                        : &startup_out.idq_ref);
  if (step_ok)
  {
    if (startup_out.mode == MOTOR_MODE_OPEN_LOOP_VF)
    {
      /* V/F 为电压指令通路：不经过 PI，直接限幅后调制。 */
      step_ok = motor_control_step_voltage(&ctx->control, &config->control, i_abc, &startup_out.vdq_ref, theta_used);
    }
    else
    {
      step_ok = motor_control_step(&ctx->control, &config->control, i_abc, theta_used, omega_used, dt_s);
    }
  }

  if (!step_ok)
  {
    ctx->status.fault_flags = (uint32_t)FAULT_INTERNAL;
    control_path_force_safe_stop(ctx);
    return false;
  }

  /* 故障判定与安全停机：观测器发散与启动失败必须锁存。 */
  ctx->status.fault_flags = 0u;
  if (ctx->observer.diverged)
  {
    ctx->status.fault_flags |= (uint32_t)FAULT_OBSERVER_DIVERGED;
  }
  if (startup_out.start_failed)
  {
    ctx->status.fault_flags |= (uint32_t)FAULT_START_FAILED;
  }
  if (!motor_control_duty_is_safe(&ctx->control))
  {
    ctx->status.fault_flags |= (uint32_t)FAULT_INTERNAL;
  }
  if (ctx->status.fault_flags != 0u)
  {
    control_path_force_safe_stop(ctx);
    ctx->status.limit_count = ctx->control.limit_count;
    return false;
  }

  /* 更新遥测状态。 */
  ctx->status.phase = ctx->startup.phase;
  ctx->status.observer = ctx->observer.active;
  ctx->status.mode = startup_out.mode;
  ctx->status.quantities.iu_a = i_abc->a;
  ctx->status.quantities.iv_a = i_abc->b;
  ctx->status.quantities.iw_a = i_abc->c;
  ctx->status.quantities.ia_a = i_ab.alpha;
  ctx->status.quantities.ib_a = i_ab.beta;
  ctx->status.quantities.id_a = ctx->control.idq.d;
  ctx->status.quantities.iq_a = ctx->control.idq.q;
  ctx->status.quantities.vd_v = ctx->control.vdq.d;
  ctx->status.quantities.vq_v = ctx->control.vdq.q;
  ctx->status.quantities.elec_angle_rad = motor_normalize_angle(theta_used);
  ctx->status.quantities.mech_speed_rpm = mech_speed_rpm;
  ctx->status.quantities.configured_bus_voltage_v = config->control.svpwm.bus_voltage_v;
  ctx->status.duty[0] = ctx->control.duty[0];
  ctx->status.duty[1] = ctx->control.duty[1];
  ctx->status.duty[2] = ctx->control.duty[2];
  ctx->status.modulation_index = ctx->control.modulation.modulation_index;
  ctx->status.theta_used_rad = motor_normalize_angle(theta_used);
  ctx->status.observer_speed_rpm = ctx->observer.speed_rpm;
  ctx->status.smo_speed_rpm = ctx->observer.smo_speed_rpm;
  ctx->status.v_limit_v = ctx->control.v_limit_v;
  ctx->status.hfi_error = ctx->startup.hfi_error;
  ctx->status.angle_error_rad = ctx->observer.angle_error_rad;
  ctx->status.limit_count = ctx->control.limit_count;
  ctx->status.fallback_count = ctx->startup.fallback_count;
  ctx->status.closed_loop = startup_out.closed_loop;
  ctx->status.start_failed = startup_out.start_failed;
  ctx->status.observer_diverged = ctx->observer.diverged;
  ctx->status.flux_monitor_ok = ctx->observer.flux_monitor_ok;
  ctx->status.limited = ctx->control.limited;
  ctx->status.duty_safe = true;
  ctx->status.stopped = false;

  return true;
}
