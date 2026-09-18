/**
 * @file motor_types.h
 * @brief 电机域公共类型定义。
 *
 * 主要接口：本文件只提供轴编号、状态、模式、方向和参数数据结构，不声明硬件访问函数。
 * 依赖关系：仅依赖 C11 固定宽度整数类型和标准浮点类型。
 * 关键安全约束：所有参数默认值都不是“已标定”值；只有 calibration_valid_mask 置位且
 * 参数 CRC、版本和范围全部通过后，软件才允许进入带功率闭环。
 */

#ifndef FOC_MOTOR_TYPES_H
#define FOC_MOTOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 参数记录格式版本；任何字段偏移变化都必须递增该版本。 */
#define MOTOR_PARAMETER_SCHEMA_VERSION ((uint16_t)1u)

/** @brief 参数记录版本，用于识别参数语义而不是文件格式。 */
#define MOTOR_PARAMETER_VERSION ((uint16_t)1u)

/** @brief 轴数量；数组必须使用 AXIS_COUNT 作为长度。 */
#define MOTOR_AXIS_COUNT ((uint32_t)2u)

/**
 * @brief 轴编号。
 *
 * 取值范围：AXIS_A、AXIS_B 或 AXIS_COUNT；AXIS_COUNT 只能用于数组边界，不能访问硬件。
 */
typedef enum {
  AXIS_A = 0,       /**< A 路：TMR1、ADC1、EG2132 功率级。 */
  AXIS_B = 1,       /**< B 路：TMR3/TMR4、ADC2、DRV8323 功率级。 */
  AXIS_COUNT = 2    /**< 轴数量哨兵。 */
} axis_id_t;

/**
 * @brief 单轴运行状态。
 *
 * 取值范围：POWER_ON 到 RECOVERY；只有 app_state_machine.c 可以修改该状态。
 */
typedef enum {
  APP_STATE_POWER_ON = 0,       /**< 上电和时钟、GPIO 安全初始化。 */
  APP_STATE_SELF_TEST,          /**< 外设、参数和安全条件自检。 */
  APP_STATE_CALIBRATION,        /**< 电流零偏和 DRV8323 校准，PWM 必须关闭。 */
  APP_STATE_IDLE,               /**< 输出关闭，等待启动命令。 */
  APP_STATE_ALIGN,              /**< 受控预定位。 */
  APP_STATE_OPEN_LOOP_IF,       /**< I/F 或 V/F 开环加速。 */
  APP_STATE_HFI_CLOSED_LOOP,    /**< 实验性 HFI 低频闭环。 */
  APP_STATE_BLEND,              /**< 开环角度与反电动势观测器连续混合。 */
  APP_STATE_CLOSED_LOOP,        /**< 正常无感闭环。 */
  APP_STATE_FIELD_WEAKENING,    /**< 受约束弱磁。 */
  APP_STATE_BRAKE,              /**< 受控制动。 */
  APP_STATE_COAST,              /**< 关闭功率输出并自由停车。 */
  APP_STATE_FAULT,              /**< 锁存故障并保持安全输出。 */
  APP_STATE_RECOVERY,           /**< 人工恢复前重新检查故障条件。 */
  APP_STATE_COUNT               /**< 状态数量哨兵。 */
} app_state_t;

/**
 * @brief 控制模式。
 *
 * 本枚举用于命令和遥测，不直接代表当前状态机状态。
 */
typedef enum {
  MOTOR_MODE_STOP = 0,            /**< 停止输出。 */
  MOTOR_MODE_OPEN_LOOP_VF,       /**< 开环 V/F。 */
  MOTOR_MODE_OPEN_LOOP_IF,       /**< 开环 I/F。 */
  MOTOR_MODE_TORQUE_IQ,          /**< Iq 转矩控制。 */
  MOTOR_MODE_CURRENT_IDIQ,       /**< Id/Iq 双电流闭环。 */
  MOTOR_MODE_SPEED,              /**< 速度闭环。 */
  MOTOR_MODE_BRAKE,              /**< 受控制动。 */
  MOTOR_MODE_COAST,              /**< 自由停车。 */
  MOTOR_MODE_IDENTIFICATION,     /**< 参数辨识。 */
  MOTOR_MODE_EXTERNAL_PWM,       /**< 外部 PWM 透传。 */
  MOTOR_MODE_COUNT               /**< 控制模式数量哨兵。 */
} motor_mode_t;

/**
 * @brief 启动策略。
 *
 * 取值范围：默认开环 I/F 或实验性 HFI；HFI 不可靠时必须允许回退 I/F。
 */
typedef enum {
  START_STRATEGY_OPEN_LOOP_IF = 0,   /**< I/F 开环启动。 */
  START_STRATEGY_HFI_EXPERIMENTAL,   /**< 实验性 HFI 启动。 */
  START_STRATEGY_COUNT               /**< 启动策略数量哨兵。 */
} start_strategy_t;

/**
 * @brief 旋转方向。
 *
 * 取值范围：FORWARD 表示机械转速和电角度增加，REVERSE 表示相反方向。
 */
typedef enum {
  MOTOR_DIRECTION_FORWARD = 0,   /**< 正转。 */
  MOTOR_DIRECTION_REVERSE        /**< 反转。 */
} motor_direction_t;

/**
 * @brief 故障严重度。
 *
 * 取值范围：INFO 到 BUS_SAFE；严重度决定恢复策略，而不是决定是否立即关闭 PWM。
 */
typedef enum {
  FAULT_SEVERITY_INFO = 0,       /**< 提示，仅记录。 */
  FAULT_SEVERITY_RECOVERABLE,    /**< 可自动恢复，但目标转矩保持为零。 */
  FAULT_SEVERITY_AXIS_LATCHED,   /**< 对应轴锁存并停机。 */
  FAULT_SEVERITY_HARDWARE,       /**< 硬件相关锁存，必须排查。 */
  FAULT_SEVERITY_BUS_SAFE        /**< 公共安全故障，两轴停机。 */
} fault_severity_t;

/**
 * @brief 故障位定义。
 *
 * 取值范围：每个常量只占用一个位；组合值可以写入 32 位原子故障位图。
 */
typedef enum {
  FAULT_OVERCURRENT_A = UINT32_C(1) << 0,       /**< A 路立即过流。 */
  FAULT_OVERCURRENT_B = UINT32_C(1) << 1,       /**< B 路立即过流。 */
  FAULT_DRV8323_NFAULT = UINT32_C(1) << 2,      /**< DRV8323 nFAULT 下降沿。 */
  FAULT_DRV8323_SPI = UINT32_C(1) << 3,         /**< DRV8323 SPI 通信或寄存器校验错误。 */
  FAULT_OBSERVER_DIVERGED = UINT32_C(1) << 4,   /**< 观测器发散或交叉检查失败。 */
  FAULT_START_FAILED = UINT32_C(1) << 5,        /**< 启动重试耗尽。 */
  FAULT_STALL = UINT32_C(1) << 6,               /**< 堵转。 */
  FAULT_PHASE_IMBALANCE = UINT32_C(1) << 7,     /**< 相电流严重不平衡。 */
  FAULT_COMM_TIMEOUT = UINT32_C(1) << 8,        /**< CAN 命令超时。 */
  FAULT_WATCHDOG_RESET = UINT32_C(1) << 9,      /**< 上次复位来自看门狗。 */
  FAULT_PARAMETER_CRC = UINT32_C(1) << 10,      /**< 参数 CRC、长度或版本非法。 */
  FAULT_ILLEGAL_STATE = UINT32_C(1) << 11,      /**< 非法状态转移。 */
  FAULT_SAMPLE_WINDOW = UINT32_C(1) << 12,      /**< ADC 采样窗口无效。 */
  FAULT_CALIBRATION_INVALID = UINT32_C(1) << 13,/**< 电流或驱动器校准无效。 */
  FAULT_INTERNAL = UINT32_C(1) << 14,            /**< 未分类内部故障。 */
  FAULT_OVER_SPEED = UINT32_C(1) << 15           /**< 超过超速故障转速（默认 7500rpm）。 */
} fault_code_t;

/**
 * @brief 故障发生时的上下文。
 *
 * 所有字段在记录时复制，调用方可以在函数返回后复用结构体。
 */
typedef struct {
  uint32_t timestamp_ms;       /**< 单调毫秒时间，单位 ms，范围 0 到 UINT32_MAX。 */
  float speed_rpm;             /**< 发生故障时的机械转速，单位 rpm。 */
  float current_a;             /**< 发生故障时的代表性相电流幅值或最大电流，单位 A。 */
  app_state_t controller_state;/**< 发生故障时的状态机状态。 */
  uint16_t firmware_version;   /**< 固件主次版本编码。 */
  uint16_t reserved;           /**< 预留字段，写入 0。 */
} fault_context_t;

/**
 * @brief 参数记录正文中的轴标定数据。
 *
 * 电流换算公式固定为 I = (voltage_v - zero_offset_v) * gain_a_per_v * polarity。
 */
typedef struct {
  float current_gain_a_per_v;  /**< 电流换算斜率，单位 A/V，必须大于 0。 */
  float current_zero_offset_v; /**< 零电流电压偏置，单位 V，目标范围 0 到 3.3V。 */
  float current_polarity;      /**< 极性，必须为 +1.0 或 -1.0。 */
} current_calibration_t;

/**
 * @brief 可持久化参数集合。
 *
 * 单位均为国际单位制；结构体只用于运行期，Flash 正文必须由 storage_parameter.c 逐字段编码。
 */
typedef struct {
  uint16_t schema_version;          /**< 参数正文格式版本。 */
  uint16_t parameter_version;       /**< 参数语义版本。 */
  uint8_t pole_pairs;               /**< 极对数，X3508 默认 7，范围 1 到 32。 */
  uint8_t calibration_valid_mask;   /**< bit0 A 路、bit1 B 路、bit2 电机参数有效。 */
  uint16_t reserved0;               /**< 预留字段，必须为 0。 */
  float rs_ohm;                     /**< 相电阻，单位 Ω；未辨识时为 0。 */
  float ld_h;                       /**< d 轴电感，单位 H；未辨识时为 0。 */
  float lq_h;                       /**< q 轴电感，单位 H；未辨识时为 0。 */
  float flux_wb;                    /**< 永磁磁链，单位 Wb；未辨识时为 0。 */
  float inertia_kg_m2;              /**< 转动惯量，单位 kg*m^2；未辨识时为 0。 */
  float torque_constant_nm_per_a;   /**< 转矩常数，单位 N*m/A；未辨识时为 0。 */
  current_calibration_t current_calibration[MOTOR_AXIS_COUNT]; /**< A/B 路电流标定。 */
  float configured_bus_voltage_v;   /**< 配置母线电压，单位 V，范围 8 到 20；不是测量值。 */
  float continuous_current_a;       /**< 连续相电流限制，单位 A，默认 12。 */
  float peak_current_a;             /**< 5 秒峰值电流限制，单位 A，默认 18。 */
  float immediate_current_a;        /**< 立即过流阈值，单位 A，默认 20。 */
  float max_speed_rpm;              /**< 最高运行转速，单位 rpm，默认 7000。 */
  float over_speed_fault_rpm;       /**< 最高故障转速，单位 rpm，默认 7500。 */
} parameter_set_t;

/**
 * @brief 20kHz 控制域和 1kHz 管理域共享的电机量。
 *
 * 所有字段必须来自同一控制周期快照，禁止跨周期拼接。
 */
typedef struct {
  float iu_a;                       /**< U 相电流，单位 A，正方向从桥臂流向电机。 */
  float iv_a;                       /**< V 相电流，单位 A。 */
  float iw_a;                       /**< 重构 W 相电流，单位 A。 */
  float ia_a;                       /**< 静止坐标 alpha 电流，单位 A。 */
  float ib_a;                       /**< 静止坐标 beta 电流，单位 A。 */
  float id_a;                       /**< d 轴电流，单位 A。 */
  float iq_a;                       /**< q 轴电流，单位 A。 */
  float vd_v;                       /**< d 轴电压，单位 V。 */
  float vq_v;                       /**< q 轴电压，单位 V。 */
  float elec_angle_rad;             /**< 电角度，单位 rad，规范到 [0, 2π)。 */
  float mech_speed_rpm;             /**< 机械速度，单位 rpm。 */
  float configured_bus_voltage_v;   /**< 配置母线电压，单位 V，不是硬件测量结果。 */
} motor_quantity_t;

/**
 * @brief 对外一致的遥测快照。
 *
 * sequence 每次发布递增；读者必须先复制快照，再比较前后 sequence 是否一致。
 */
typedef struct {
  uint32_t sequence;                /**< 快照序号，用于一致性判断。 */
  uint32_t timestamp_ms;            /**< 快照时间戳，单位 ms。 */
  app_state_t state;                /**< 快照所属轴的状态。 */
  motor_mode_t mode;                /**< 当前控制模式。 */
  motor_quantity_t quantities;      /**< 同一周期的电机量。 */
  uint32_t fault_flags;             /**< 当前故障位图。 */
  uint16_t parameter_version;       /**< 生效参数版本。 */
  uint16_t reserved;                /**< 预留字段，发布时写 0。 */
} telemetry_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif
