/**
 * @file comm_guard.c
 * @brief 通信与日志隔离、CAN 超时看门狗和日志节流实现。
 *
 * 本文件为纯逻辑实现，不访问寄存器、不分配内存，可在主机测试。
 * 隔离原则：每个通道独立维护 enabled / fault_latched / 计数；
 * 任一通道故障只影响自身，CAN 超时只触发控制侧转矩归零决策。
 */

#include "comm/comm_guard.h"

/** @brief 通道状态。 */
typedef struct {
  bool enabled;          /**< 通道是否启用。 */
  bool fault_latched;    /**< 通道是否锁存故障。 */
  uint32_t tx_dropped;   /**< 因禁用或故障丢弃的发送字节数。 */
  uint32_t rx_overflow;  /**< 因禁用或故障丢弃的接收字节数。 */
  uint32_t last_tx_ms;   /**< 最近一次允许发送的毫秒时间。 */
  uint32_t last_rx_ms;   /**< 最近一次允许接收的毫秒时间。 */
} comm_channel_state_t;

/** @brief 全部通道状态，静态分配。 */
static comm_channel_state_t g_channels[COMM_CHANNEL_COUNT];

/** @brief CAN 最近一次喂狗毫秒时间。 */
static uint32_t g_can_last_feed_ms;

/** @brief CAN 是否已喂过狗。 */
static bool g_can_fed;

/** @brief 日志最近一次放行毫秒时间。 */
static uint32_t g_log_last_allowed_ms;

/** @brief 日志是否已放行过。 */
static bool g_log_allowed_once;

/** @brief 日志被节流丢弃的条数。 */
static uint32_t g_log_dropped;

/**
 * @brief 校验通道编号。
 *
 * @param channel 通道编号。
 * @return 合法返回 true。
 */
static bool comm_channel_valid(comm_channel_t channel)
{
  return ((uint32_t)channel < COMM_CHANNEL_COUNT);
}

/**
 * @brief 初始化全部通道为启用且健康。
 *
 * @return 无返回值。
 *
 * 调用上下文：系统初始化。
 * 失败行为：无失败路径。
 */
void comm_guard_init(void)
{
  uint32_t index;

  for (index = 0u; index < COMM_CHANNEL_COUNT; ++index) {
    g_channels[index].enabled = true;
    g_channels[index].fault_latched = false;
    g_channels[index].tx_dropped = 0u;
    g_channels[index].rx_overflow = 0u;
    g_channels[index].last_tx_ms = 0u;
    g_channels[index].last_rx_ms = 0u;
  }

  g_can_last_feed_ms = 0u;
  g_can_fed = false;
  g_log_last_allowed_ms = 0u;
  g_log_allowed_once = false;
  g_log_dropped = 0u;
}

/**
 * @brief 启用或禁用通道。
 *
 * @param channel 通道编号。
 * @param enabled true 表示允许收发。
 * @return true 表示设置成功。
 */
bool comm_guard_set_channel_enabled(comm_channel_t channel, bool enabled)
{
  if (!comm_channel_valid(channel)) {
    return false;
  }

  g_channels[(uint32_t)channel].enabled = enabled;
  return true;
}

/**
 * @brief 查询通道是否启用。
 *
 * @param channel 通道编号。
 * @return true 表示启用。
 */
bool comm_guard_is_channel_enabled(comm_channel_t channel)
{
  if (!comm_channel_valid(channel)) {
    return false;
  }

  return g_channels[(uint32_t)channel].enabled;
}

/**
 * @brief 锁存指定通道故障。
 *
 * @param channel 通道编号。
 * @return 无返回值。
 */
void comm_guard_channel_fault(comm_channel_t channel)
{
  if (comm_channel_valid(channel)) {
    g_channels[(uint32_t)channel].fault_latched = true;
  }
}

/**
 * @brief 清除指定通道锁存故障。
 *
 * @param channel 通道编号。
 * @return 无返回值。
 */
void comm_guard_channel_healthy(comm_channel_t channel)
{
  if (comm_channel_valid(channel)) {
    g_channels[(uint32_t)channel].fault_latched = false;
  }
}

/**
 * @brief 查询通道是否健康。
 *
 * @param channel 通道编号。
 * @return true 表示启用且无锁存故障。
 */
bool comm_guard_is_channel_healthy(comm_channel_t channel)
{
  if (!comm_channel_valid(channel)) {
    return false;
  }

  return g_channels[(uint32_t)channel].enabled &&
         !g_channels[(uint32_t)channel].fault_latched;
}

/**
 * @brief 记录一次发送。
 *
 * @param channel 通道编号。
 * @param now_ms 当前单调毫秒时间。
 * @param byte_count 本次字节数。
 * @return true 表示允许发送。
 */
bool comm_guard_tx_record(comm_channel_t channel, uint32_t now_ms, uint32_t byte_count)
{
  comm_channel_state_t *state;

  if (!comm_channel_valid(channel)) {
    return false;
  }

  state = &g_channels[(uint32_t)channel];
  if (!state->enabled || state->fault_latched) {
    state->tx_dropped += byte_count;
    return false;
  }

  state->last_tx_ms = now_ms;
  return true;
}

/**
 * @brief 记录一次接收。
 *
 * @param channel 通道编号。
 * @param now_ms 当前单调毫秒时间。
 * @param byte_count 本次字节数。
 * @return true 表示允许处理。
 */
bool comm_guard_rx_record(comm_channel_t channel, uint32_t now_ms, uint32_t byte_count)
{
  comm_channel_state_t *state;

  if (!comm_channel_valid(channel)) {
    return false;
  }

  state = &g_channels[(uint32_t)channel];
  if (!state->enabled || state->fault_latched) {
    state->rx_overflow += byte_count;
    return false;
  }

  state->last_rx_ms = now_ms;
  return true;
}

/**
 * @brief 读取通道发送丢弃计数。
 *
 * @param channel 通道编号。
 * @return 丢弃字节数。
 */
uint32_t comm_guard_tx_dropped(comm_channel_t channel)
{
  if (!comm_channel_valid(channel)) {
    return 0u;
  }

  return g_channels[(uint32_t)channel].tx_dropped;
}

/**
 * @brief 读取通道接收溢出计数。
 *
 * @param channel 通道编号。
 * @return 溢出字节数。
 */
uint32_t comm_guard_rx_overflow(comm_channel_t channel)
{
  if (!comm_channel_valid(channel)) {
    return 0u;
  }

  return g_channels[(uint32_t)channel].rx_overflow;
}

/**
 * @brief 记录 CAN 心跳帧。
 *
 * @param now_ms 当前单调毫秒时间。
 * @return 无返回值。
 */
void comm_guard_can_feed(uint32_t now_ms)
{
  g_can_last_feed_ms = now_ms;
  g_can_fed = true;
}

/**
 * @brief 检查 CAN 是否超时。
 *
 * @param now_ms 当前单调毫秒时间。
 * @param timeout_ms 允许的最大间隔，单位 ms。
 * @return true 表示超时。
 */
bool comm_guard_can_timeout_check(uint32_t now_ms, uint32_t timeout_ms)
{
  uint32_t elapsed_ms;

  if (!g_can_fed || (timeout_ms == 0u)) {
    return true;
  }

  elapsed_ms = (uint32_t)(now_ms - g_can_last_feed_ms);
  return elapsed_ms >= timeout_ms;
}

/**
 * @brief 日志节流。
 *
 * @param now_ms 当前单调毫秒时间。
 * @param min_interval_ms 最小放行间隔，单位 ms。
 * @return true 表示允许写日志。
 */
bool comm_guard_log_allowed(uint32_t now_ms, uint32_t min_interval_ms)
{
  uint32_t elapsed_ms;

  if (!g_log_allowed_once) {
    g_log_last_allowed_ms = now_ms;
    g_log_allowed_once = true;
    return true;
  }

  if (min_interval_ms == 0u) {
    g_log_last_allowed_ms = now_ms;
    return true;
  }

  elapsed_ms = (uint32_t)(now_ms - g_log_last_allowed_ms);
  if (elapsed_ms >= min_interval_ms) {
    g_log_last_allowed_ms = now_ms;
    return true;
  }

  g_log_dropped++;
  return false;
}

/**
 * @brief 读取日志丢弃计数。
 *
 * @return 丢弃条数。
 */
uint32_t comm_guard_log_dropped(void)
{
  return g_log_dropped;
}
