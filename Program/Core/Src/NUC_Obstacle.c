/**
 * @file NUC_Obstacle.c
 * @brief NUC navigation UART transport
 *
 * Frame: AA BB | XH XL | YH YL | CHK | 55
 * Coordinate: X forward (mm), Y lateral (mm, positive=left)
 */
#include "NUC_Obstacle.h"
#include "STP23L.h"
#include <string.h>

#define NUC_FRAME_HEADER1  0xAA
#define NUC_FRAME_HEADER2  0xBB
#define NUC_FRAME_TAIL     0x55
#define NUC_FRAME_LEN      8U
#define NUC_TIMEOUT_MS     200U
#define NUC_FRAME_GAP_MS   50U
#define NUC_HANDSHAKE_GAP_MS 100U
#define NUC_HANDSHAKE_LEN    4U

#define NAV_SYNC_1           0xAAU
#define NAV_SYNC_2           0xCCU
#define NAV_VERSION          1U
#define NAV_MAX_PAYLOAD      48U
#define NAV_FRAME_MAX_LEN    (2U + 4U + NAV_MAX_PAYLOAD + 2U)
#define NAV_ONLINE_MS        500U
#define NAV_VELOCITY_TIMEOUT_MS 250U
#define NAV_STM32_MAX_LINEAR_MM_S 2000
#define NAV_POSE_PERIOD_MS   20U
#define NAV_GOAL_PERIOD_MS   250U
#define NAV_STP23L_PERIOD_MS 100U

#define NAV_MSG_POSE         0x10U
#define NAV_MSG_GOAL_REQUEST 0x11U
#define NAV_MSG_NAV_STATUS   0x12U
#define NAV_MSG_STP23L       0x13U
#define NAV_MSG_SCAN_ACK     0x14U
#define NAV_MSG_PATH_BEGIN   0x20U
#define NAV_MSG_WAYPOINT     0x21U
#define NAV_MSG_PATH_COMMIT  0x22U
#define NAV_MSG_PATH_CANCEL  0x23U
#define NAV_MSG_HEARTBEAT    0x30U
#define NAV_MSG_VEL_CMD      0x40U
#define NAV_MSG_SCAN_RESULT  0x41U

static uint8_t s_it_byte;
static uint8_t s_buf[NUC_FRAME_LEN];
static uint8_t s_idx;
static uint8_t s_pong_idx;
static volatile int16_t s_obstacle_x;
static volatile int16_t s_obstacle_y;
static volatile uint32_t s_last_valid_ms;
static const uint8_t s_ping[NUC_HANDSHAKE_LEN] = {0xA5U, 0x5AU, 0x01U, 0x00U};
static const uint8_t s_pong[NUC_HANDSHAKE_LEN] = {0xA5U, 0x5AU, 0x01U, 0x01U};

/* Debug: expose frame buffer for inspection */
volatile uint8_t g_nuc_debug_buf[8];
volatile uint8_t g_nuc_parse_fail_reason = 0; // 1=header seen, 2=tail, 3=checksum, 0xFF=success
volatile uint32_t g_nuc_rx_count = 0; // Total bytes received
volatile uint8_t g_nuc_last_byte = 0; // Last received byte
volatile uint32_t g_nuc_uart_error_count = 0;
volatile uint32_t g_nuc_last_uart_error = 0;
volatile uint8_t g_nuc_online = 0;

/* Debug: circular buffer for last 16 bytes */
#define RX_HISTORY_SIZE 16
volatile uint8_t g_nuc_rx_history[RX_HISTORY_SIZE];
volatile uint8_t g_nuc_history_idx = 0;

static uint32_t s_last_rx_ms;

typedef struct {
  uint16_t path_id;
  uint16_t request_id;
  uint8_t goal_id;
  uint8_t count;
  NUC_NavWaypoint waypoints[NUC_NAV_MAX_WAYPOINTS];
} NUC_NavPathSlot;

static uint8_t s_nav_buf[NAV_FRAME_MAX_LEN];
static uint8_t s_nav_idx;
static uint8_t s_nav_expected_len;
static NUC_NavPathSlot s_nav_paths[2];
static volatile uint8_t s_nav_active_slot;
static volatile uint8_t s_nav_active_valid;
static uint8_t s_nav_staging_slot;
static uint32_t s_nav_staging_mask;
static uint8_t s_nav_staging_valid;
static volatile uint16_t s_nav_generation;
static volatile uint32_t s_nav_last_valid_ms;
static uint8_t s_nav_tx_seq;
static uint32_t s_nav_last_pose_tx_ms;
static uint32_t s_nav_last_goal_tx_ms;
static uint32_t s_nav_last_stp23l_tx_ms;
static NUC_NavGoal s_nav_requested_goal;
static uint16_t s_nav_request_id;
static volatile int16_t s_nav_forward_mm_s;
static volatile int16_t s_nav_left_mm_s;
static volatile int16_t s_nav_yaw_ccw_cdeg_s;
static volatile uint16_t s_nav_velocity_stamp_cs;
static volatile uint32_t s_nav_last_velocity_ms;
static volatile NUC_NavStatus s_nav_status;
static NUC_NavScanResult s_nav_scan_result;
static volatile uint8_t s_nav_scan_pending;

/* Final linear-speed guard on the STM32 side. The NUC currently limits each
 * body-axis component to 1000 mm/s, while this lower-level guard allows up to
 * 2.00 m/s for any future command source. */
static int16_t clamp_nav_linear_speed(int16_t speed_mm_s)
{
  if (speed_mm_s > NAV_STM32_MAX_LINEAR_MM_S)
  {
    return NAV_STM32_MAX_LINEAR_MM_S;
  }
  if (speed_mm_s < -NAV_STM32_MAX_LINEAR_MM_S)
  {
    return -NAV_STM32_MAX_LINEAR_MM_S;
  }
  return speed_mm_s;
}

static uint16_t read_u16_be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t read_i16_be(const uint8_t *data)
{
  return (int16_t)read_u16_be(data);
}

static int32_t read_i32_be(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];
  return (int32_t)value;
}

static void write_u16_be(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

static void write_i16_be(uint8_t *data, int16_t value)
{
  write_u16_be(data, (uint16_t)value);
}

static void write_i32_be(uint8_t *data, int32_t value)
{
  uint32_t raw = (uint32_t)value;
  data[0] = (uint8_t)(raw >> 24);
  data[1] = (uint8_t)(raw >> 16);
  data[2] = (uint8_t)(raw >> 8);
  data[3] = (uint8_t)raw;
}

static uint16_t nav_crc16(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFFU;
  uint8_t i;
  uint8_t bit;

  for (i = 0U; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                            : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static HAL_StatusTypeDef nav_send_frame(uint8_t type, const uint8_t *payload,
                                        uint8_t payload_len)
{
  uint8_t frame[NAV_FRAME_MAX_LEN];
  uint8_t body_len;
  uint8_t frame_len;
  uint16_t crc;

  if (payload_len > NAV_MAX_PAYLOAD)
  {
    return HAL_ERROR;
  }

  frame[0] = NAV_SYNC_1;
  frame[1] = NAV_SYNC_2;
  frame[2] = NAV_VERSION;
  frame[3] = type;
  frame[4] = s_nav_tx_seq++;
  frame[5] = payload_len;
  if (payload_len > 0U && payload != NULL)
  {
    memcpy(&frame[6], payload, payload_len);
  }
  body_len = (uint8_t)(4U + payload_len);
  crc = nav_crc16(&frame[2], body_len);
  frame[6U + payload_len] = (uint8_t)(crc >> 8);
  frame[7U + payload_len] = (uint8_t)crc;
  frame_len = (uint8_t)(8U + payload_len);
  return HAL_UART_Transmit(&huart8, frame, frame_len, 10U);
}

static void nav_parse_frame(void)
{
  uint8_t type = s_nav_buf[3];
  uint8_t payload_len = s_nav_buf[5];
  const uint8_t *payload = &s_nav_buf[6];
  uint16_t received_crc = read_u16_be(&s_nav_buf[6U + payload_len]);
  uint16_t calculated_crc = nav_crc16(&s_nav_buf[2], (uint8_t)(4U + payload_len));

  if (s_nav_buf[2] != NAV_VERSION || received_crc != calculated_crc)
  {
    return;
  }
  s_nav_last_valid_ms = HAL_GetTick();
  g_nuc_online = 1U;

  if (type == NAV_MSG_PATH_BEGIN && payload_len == 6U)
  {
    uint8_t count = payload[5];
    if (count == 0U || count > NUC_NAV_MAX_WAYPOINTS)
    {
      s_nav_staging_valid = 0U;
      return;
    }
    s_nav_staging_slot = (uint8_t)(s_nav_active_slot ^ 1U);
    s_nav_paths[s_nav_staging_slot].path_id = read_u16_be(&payload[0]);
    s_nav_paths[s_nav_staging_slot].request_id = read_u16_be(&payload[2]);
    s_nav_paths[s_nav_staging_slot].goal_id = payload[4];
    s_nav_paths[s_nav_staging_slot].count = count;
    s_nav_staging_mask = 0U;
    s_nav_staging_valid = 1U;
  }
  else if (type == NAV_MSG_WAYPOINT && payload_len == 15U && s_nav_staging_valid)
  {
    uint16_t path_id = read_u16_be(&payload[0]);
    uint8_t index = payload[2];
    NUC_NavPathSlot *slot = &s_nav_paths[s_nav_staging_slot];
    if (path_id != slot->path_id || index >= slot->count)
    {
      return;
    }
    slot->waypoints[index].x_mm = read_i32_be(&payload[3]);
    slot->waypoints[index].y_mm = read_i32_be(&payload[7]);
    slot->waypoints[index].yaw_cdeg = read_i16_be(&payload[11]);
    slot->waypoints[index].speed_mm_s = read_u16_be(&payload[13]);
    s_nav_staging_mask |= (uint32_t)1U << index;
  }
  else if (type == NAV_MSG_PATH_COMMIT && payload_len == 3U && s_nav_staging_valid)
  {
    NUC_NavPathSlot *slot = &s_nav_paths[s_nav_staging_slot];
    uint16_t path_id = read_u16_be(&payload[0]);
    uint8_t count = payload[2];
    uint32_t required_mask = ((uint32_t)1U << count) - 1U;
    if (path_id == slot->path_id && count == slot->count &&
        s_nav_staging_mask == required_mask &&
        slot->request_id == s_nav_request_id &&
        slot->goal_id == (uint8_t)s_nav_requested_goal)
    {
      s_nav_active_slot = s_nav_staging_slot;
      s_nav_generation++;
      if (s_nav_generation == 0U)
      {
        s_nav_generation = 1U;
      }
      s_nav_active_valid = 1U;
    }
    s_nav_staging_valid = 0U;
  }
  else if (type == NAV_MSG_PATH_CANCEL)
  {
    s_nav_staging_valid = 0U;
    s_nav_active_valid = 0U;
  }
  else if (type == NAV_MSG_NAV_STATUS && payload_len == 4U)
  {
    uint16_t request_id = read_u16_be(&payload[0]);
    uint8_t goal_id = payload[2];
    uint8_t status = payload[3];
    if (request_id == s_nav_request_id &&
        goal_id == (uint8_t)s_nav_requested_goal &&
        status <= (uint8_t)NUC_NAV_ERROR)
    {
      s_nav_status = (NUC_NavStatus)status;
    }
  }
  else if (type == NAV_MSG_VEL_CMD && payload_len == 8U)
  {
    s_nav_forward_mm_s = read_i16_be(&payload[0]);
    s_nav_left_mm_s = read_i16_be(&payload[2]);
    s_nav_yaw_ccw_cdeg_s = read_i16_be(&payload[4]);
    s_nav_velocity_stamp_cs = read_u16_be(&payload[6]);
    s_nav_last_velocity_ms = HAL_GetTick();
  }
  else if (type == NAV_MSG_SCAN_RESULT && payload_len >= 6U)
  {
    uint8_t code_len = payload[4];
    uint8_t i;

    if (code_len == 0U || code_len >= NUC_NAV_SCAN_CODE_MAX ||
        payload_len != (uint8_t)(5U + code_len) ||
        payload[2] < (uint8_t)NUC_SCAN_CONTEXT_ORDER ||
        payload[2] > (uint8_t)NUC_SCAN_CONTEXT_BED3 ||
        payload[3] < (uint8_t)NUC_SCAN_FORMAT_QR ||
        payload[3] > (uint8_t)NUC_SCAN_FORMAT_CODE128)
    {
      return;
    }
    for (i = 0U; i < code_len; i++)
    {
      if (payload[5U + i] < 0x20U || payload[5U + i] > 0x7EU)
      {
        return;
      }
    }

    s_nav_scan_result.scan_id = read_u16_be(&payload[0]);
    s_nav_scan_result.context = payload[2];
    s_nav_scan_result.format = payload[3];
    s_nav_scan_result.length = code_len;
    memcpy(s_nav_scan_result.value, &payload[5], code_len);
    s_nav_scan_result.value[code_len] = '\0';
    s_nav_scan_pending = 1U;
  }
}

static void nav_feed_byte(uint8_t value)
{
  if (s_nav_idx == 0U)
  {
    if (value == NAV_SYNC_1)
    {
      s_nav_buf[s_nav_idx++] = value;
    }
    return;
  }
  if (s_nav_idx == 1U)
  {
    if (value == NAV_SYNC_2)
    {
      s_nav_buf[s_nav_idx++] = value;
    }
    else if (value == NAV_SYNC_1)
    {
      s_nav_buf[0] = value;
      s_nav_idx = 1U;
    }
    else
    {
      s_nav_idx = 0U;
    }
    return;
  }

  if (s_nav_idx >= NAV_FRAME_MAX_LEN)
  {
    s_nav_idx = 0U;
    return;
  }
  s_nav_buf[s_nav_idx++] = value;

  if (s_nav_idx == 6U)
  {
    if (s_nav_buf[2] != NAV_VERSION || s_nav_buf[5] > NAV_MAX_PAYLOAD)
    {
      s_nav_idx = 0U;
      return;
    }
    s_nav_expected_len = (uint8_t)(8U + s_nav_buf[5]);
  }
  if (s_nav_expected_len != 0U && s_nav_idx >= s_nav_expected_len)
  {
    nav_parse_frame();
    s_nav_idx = 0U;
    s_nav_expected_len = 0U;
  }
}

static void nuc_feed_handshake_byte(uint8_t b)
{
  if (b == s_pong[s_pong_idx])
  {
    s_pong_idx++;
    if (s_pong_idx >= NUC_HANDSHAKE_LEN)
    {
      g_nuc_online = 1U;
      s_pong_idx = 0U;
    }
  }
  else if (b == s_pong[0])
  {
    s_pong_idx = 1U;
  }
  else
  {
    s_pong_idx = 0U;
  }
}

static void nuc_parse_frame(void)
{
  uint8_t chk_calc;
  uint8_t chk_recv;
  int16_t x, y;

  /* Copy frame for debugging */
  for (uint8_t i = 0; i < NUC_FRAME_LEN; i++)
  {
    g_nuc_debug_buf[i] = s_buf[i];
  }

  /* Verify header and tail */
  if (s_buf[0] != NUC_FRAME_HEADER1 || s_buf[1] != NUC_FRAME_HEADER2)
  {
    g_nuc_parse_fail_reason = 1;
    return;
  }
  if (s_buf[7] != NUC_FRAME_TAIL)
  {
    g_nuc_parse_fail_reason = 2;
    return;
  }

  /* Verify checksum */
  chk_calc = (uint8_t)(s_buf[2] + s_buf[3] + s_buf[4] + s_buf[5]);
  chk_recv = s_buf[6];
  if (chk_calc != chk_recv)
  {
    g_nuc_parse_fail_reason = 3;
    return;
  }

  /* Parse X and Y (big-endian int16) */
  x = (int16_t)(((uint16_t)s_buf[2] << 8) | s_buf[3]);
  y = (int16_t)(((uint16_t)s_buf[4] << 8) | s_buf[5]);

  s_obstacle_x = x;
  s_obstacle_y = y;
  s_last_valid_ms = HAL_GetTick();
  g_nuc_parse_fail_reason = 0xFF; // Success
}

static void nuc_feed_byte(uint8_t b)
{
  uint32_t now = HAL_GetTick();

  /* A partial frame cannot span a long bus gap; resync before consuming b. */
  if (s_idx != 0U && (now - s_last_rx_ms) > NUC_FRAME_GAP_MS)
  {
    s_idx = 0;
  }
  s_last_rx_ms = now;

  /* State 0: wait for header1 */
  if (s_idx == 0)
  {
    if (b == NUC_FRAME_HEADER1)
    {
      g_nuc_parse_fail_reason = 1;
      s_buf[s_idx++] = b;
    }
    return;
  }

  /* State 1: wait for header2 */
  if (s_idx == 1)
  {
    if (b == NUC_FRAME_HEADER2)
    {
      s_buf[s_idx++] = b;
    }
    else if (b == NUC_FRAME_HEADER1)
    {
      g_nuc_parse_fail_reason = 1;
      s_buf[0] = b;
      s_idx = 1;
    }
    else
    {
      s_idx = 0;
    }
    return;
  }

  /* State 2-7: collect remaining bytes */
  s_buf[s_idx++] = b;
  if (s_idx >= NUC_FRAME_LEN)
  {
    nuc_parse_frame();
    s_idx = 0;
  }
}

void NUC_Obstacle_Init(void)
{
  s_idx = 0;
  s_pong_idx = 0;
  s_last_rx_ms = 0;
  s_obstacle_x = 0;
  s_obstacle_y = 0;
  s_last_valid_ms = 0;
  g_nuc_parse_fail_reason = 0;
  g_nuc_rx_count = 0;
  g_nuc_last_byte = 0;
  g_nuc_uart_error_count = 0;
  g_nuc_last_uart_error = 0;
  g_nuc_online = 0;
  s_nav_idx = 0U;
  s_nav_expected_len = 0U;
  s_nav_active_slot = 0U;
  s_nav_active_valid = 0U;
  s_nav_staging_slot = 1U;
  s_nav_staging_mask = 0U;
  s_nav_staging_valid = 0U;
  s_nav_generation = 0U;
  s_nav_last_valid_ms = 0U;
  s_nav_tx_seq = 0U;
  s_nav_last_pose_tx_ms = 0U;
  s_nav_last_goal_tx_ms = 0U;
  s_nav_last_stp23l_tx_ms = 0U;
  s_nav_requested_goal = NUC_NAV_GOAL_NONE;
  s_nav_request_id = 0U;
  s_nav_forward_mm_s = 0;
  s_nav_left_mm_s = 0;
  s_nav_yaw_ccw_cdeg_s = 0;
  s_nav_velocity_stamp_cs = 0U;
  s_nav_last_velocity_ms = 0U;
  s_nav_status = NUC_NAV_IDLE;
  s_nav_scan_pending = 0U;
  memset((void *)g_nuc_debug_buf, 0, sizeof(g_nuc_debug_buf));
  memset((void *)g_nuc_rx_history, 0, sizeof(g_nuc_rx_history));
  g_nuc_history_idx = 0;
  memset(s_buf, 0, sizeof(s_buf));
  memset(s_nav_buf, 0, sizeof(s_nav_buf));
  memset(s_nav_paths, 0, sizeof(s_nav_paths));
  memset(&s_nav_scan_result, 0, sizeof(s_nav_scan_result));
  (void)HAL_UART_AbortReceive_IT(&huart8);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

HAL_StatusTypeDef NUC_Obstacle_SendPing(void)
{
  return HAL_UART_Transmit(&huart8, s_ping, NUC_HANDSHAKE_LEN, 10U);
}

uint8_t NUC_IsOnline(void)
{
  return g_nuc_online;
}

uint8_t NUC_Obstacle_WaitForOnline(uint32_t timeout_ms)
{
  uint32_t start_ms = HAL_GetTick();
  uint32_t last_ping_ms = start_ms - NUC_HANDSHAKE_GAP_MS;

  g_nuc_online = 0U;
  s_pong_idx = 0U;

  while ((HAL_GetTick() - start_ms) < timeout_ms)
  {
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_ping_ms) >= NUC_HANDSHAKE_GAP_MS)
    {
      (void)NUC_Obstacle_SendPing();
      last_ping_ms = now_ms;
    }

    if (NUC_IsOnline() != 0U)
    {
      return 1U;
    }

    HAL_Delay(1U);
  }

  return NUC_IsOnline();
}

void NUC_Obstacle_OnUartError(void)
{
  g_nuc_uart_error_count++;
  g_nuc_last_uart_error = huart8.ErrorCode;

  /* ORE/RTO abort interrupt reception; restart it after HAL marks the
     receive state ready. Frame/noise errors are recoverable in HAL and keep
     the current one-byte reception active. */
  if ((huart8.ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)) != 0U)
  {
    (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
  }
}

void NUC_Obstacle_OnUartRxCplt(void)
{
  g_nuc_rx_count++;
  g_nuc_last_byte = s_it_byte;

  // Store in circular history buffer
  g_nuc_rx_history[g_nuc_history_idx] = s_it_byte;
  g_nuc_history_idx = (g_nuc_history_idx + 1) % RX_HISTORY_SIZE;

  nuc_feed_handshake_byte(s_it_byte);
  nuc_feed_byte(s_it_byte);
  nav_feed_byte(s_it_byte);
  (void)HAL_UART_Receive_IT(&huart8, &s_it_byte, 1);
}

int16_t NUC_GetObstacleX(void)
{
  if (!NUC_IsObstacleValid())
  {
    return 0;
  }
  return s_obstacle_x;
}

int16_t NUC_GetObstacleY(void)
{
  if (!NUC_IsObstacleValid())
  {
    return 0;
  }
  return s_obstacle_y;
}

uint8_t NUC_IsObstacleValid(void)
{
  if (s_last_valid_ms == 0)
  {
    return 0;
  }
  return (HAL_GetTick() - s_last_valid_ms) < NUC_TIMEOUT_MS ? 1U : 0U;
}

void NUC_Nav_RequestGoal(NUC_NavGoal goal)
{
  if (goal == NUC_NAV_GOAL_NONE)
  {
    return;
  }
  if (goal != s_nav_requested_goal)
  {
    s_nav_requested_goal = goal;
    s_nav_status = NUC_NAV_WAIT_PATH;
    s_nav_request_id++;
    if (s_nav_request_id == 0U)
    {
      s_nav_request_id = 1U;
    }
    s_nav_last_goal_tx_ms = 0U;
    s_nav_staging_valid = 0U;
  }
}

void NUC_Nav_Service(int32_t x_mm, int32_t y_mm, int16_t yaw_cdeg,
                     uint8_t task_state, NUC_NavStatus nav_status,
                     uint16_t path_id, uint8_t waypoint_index)
{
  uint32_t now = HAL_GetTick();

  /* If the NUC/ROS stack restarts during a goal, stop immediately and put the
   * same request back on the wire once the serial bridge returns. */
  if (s_nav_requested_goal != NUC_NAV_GOAL_NONE &&
      s_nav_status == NUC_NAV_FOLLOWING &&
      (s_nav_last_valid_ms == 0U ||
       (now - s_nav_last_valid_ms) >= NAV_ONLINE_MS))
  {
    s_nav_status = NUC_NAV_WAIT_PATH;
    nav_status = NUC_NAV_WAIT_PATH;
    s_nav_last_goal_tx_ms = 0U;
    NUC_Nav_ClearVelocity();
  }

  if ((now - s_nav_last_pose_tx_ms) >= NAV_POSE_PERIOD_MS)
  {
    uint8_t payload[15];
    write_i32_be(&payload[0], x_mm);
    write_i32_be(&payload[4], y_mm);
    write_i16_be(&payload[8], yaw_cdeg);
    payload[10] = task_state;
    payload[11] = (uint8_t)nav_status;
    write_u16_be(&payload[12], path_id);
    payload[14] = waypoint_index;
    (void)nav_send_frame(NAV_MSG_POSE, payload, sizeof(payload));
    s_nav_last_pose_tx_ms = now;
  }

  if (s_nav_requested_goal != NUC_NAV_GOAL_NONE &&
      s_nav_status == NUC_NAV_WAIT_PATH &&
      (s_nav_last_goal_tx_ms == 0U || (now - s_nav_last_goal_tx_ms) >= NAV_GOAL_PERIOD_MS))
  {
    uint8_t payload[3];
    write_u16_be(&payload[0], s_nav_request_id);
    payload[2] = (uint8_t)s_nav_requested_goal;
    (void)nav_send_frame(NAV_MSG_GOAL_REQUEST, payload, sizeof(payload));
    s_nav_last_goal_tx_ms = now;
  }
}

void NUC_Nav_ServiceSTP23L(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t payload[7];
  uint8_t valid_mask = 0U;

  if ((now - s_nav_last_stp23l_tx_ms) < NAV_STP23L_PERIOD_MS)
  {
    return;
  }

  if (STP23L_IsOnlineA() != 0U)
  {
    valid_mask |= 0x01U;
  }
  if (STP23L_IsOnlineB() != 0U)
  {
    valid_mask |= 0x02U;
  }
  if (STP23L_IsOnlineC() != 0U)
  {
    valid_mask |= 0x04U;
  }

  /* A=right, B=front, C=left. Distances are sensor-face millimetres. */
  write_u16_be(&payload[0], STP32_getA());
  write_u16_be(&payload[2], STP32_getB());
  write_u16_be(&payload[4], STP32_getC());
  payload[6] = valid_mask;
  (void)nav_send_frame(NAV_MSG_STP23L, payload, sizeof(payload));
  s_nav_last_stp23l_tx_ms = now;
}

uint8_t NUC_Nav_HasRequestedPath(void)
{
  uint8_t slot = s_nav_active_slot;
  return (s_nav_active_valid != 0U &&
          s_nav_paths[slot].request_id == s_nav_request_id &&
          s_nav_paths[slot].goal_id == (uint8_t)s_nav_requested_goal) ? 1U : 0U;
}

uint16_t NUC_Nav_GetPathGeneration(void)
{
  return s_nav_generation;
}

uint16_t NUC_Nav_GetPathId(void)
{
  return s_nav_paths[s_nav_active_slot].path_id;
}

uint8_t NUC_Nav_GetPathCount(void)
{
  return s_nav_paths[s_nav_active_slot].count;
}

uint8_t NUC_Nav_GetWaypoint(uint8_t index, NUC_NavWaypoint *waypoint)
{
  uint8_t slot;
  uint16_t generation_before;
  uint16_t generation_after;

  if (waypoint == NULL)
  {
    return 0U;
  }
  do
  {
    generation_before = s_nav_generation;
    slot = s_nav_active_slot;
    if (generation_before == 0U || index >= s_nav_paths[slot].count)
    {
      return 0U;
    }
    *waypoint = s_nav_paths[slot].waypoints[index];
    generation_after = s_nav_generation;
  } while (generation_before != generation_after);
  return 1U;
}

uint8_t NUC_Nav_GetVelocityCommand(int16_t *forward_mm_s,
                                   int16_t *left_mm_s,
                                   int16_t *yaw_ccw_cdeg_s)
{
  uint32_t primask;
  uint32_t last_ms;

  if (forward_mm_s == NULL || left_mm_s == NULL || yaw_ccw_cdeg_s == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  last_ms = s_nav_last_velocity_ms;
  *forward_mm_s = clamp_nav_linear_speed(s_nav_forward_mm_s);
  *left_mm_s = clamp_nav_linear_speed(s_nav_left_mm_s);
  *yaw_ccw_cdeg_s = s_nav_yaw_ccw_cdeg_s;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (last_ms == 0U || (HAL_GetTick() - last_ms) > NAV_VELOCITY_TIMEOUT_MS)
  {
    *forward_mm_s = 0;
    *left_mm_s = 0;
    *yaw_ccw_cdeg_s = 0;
    return 0U;
  }
  return 1U;
}

void NUC_Nav_ClearVelocity(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  s_nav_forward_mm_s = 0;
  s_nav_left_mm_s = 0;
  s_nav_yaw_ccw_cdeg_s = 0;
  s_nav_velocity_stamp_cs = 0U;
  s_nav_last_velocity_ms = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

uint8_t NUC_Nav_TakeScanResult(NUC_NavScanResult *result)
{
  uint32_t primask;

  if (result == NULL)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (s_nav_scan_pending == 0U)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }
  *result = s_nav_scan_result;
  s_nav_scan_pending = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

void NUC_Nav_ClearScanResult(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  s_nav_scan_pending = 0U;
  memset(&s_nav_scan_result, 0, sizeof(s_nav_scan_result));
  if (primask == 0U)
  {
    __enable_irq();
  }
}

HAL_StatusTypeDef NUC_Nav_SendScanAck(uint16_t scan_id,
                                      NUC_ScanAckStatus status)
{
  uint8_t payload[3];

  if (status < NUC_SCAN_ACK_ACCEPTED || status > NUC_SCAN_ACK_INVALID_CODE)
  {
    return HAL_ERROR;
  }
  write_u16_be(&payload[0], scan_id);
  payload[2] = (uint8_t)status;
  return nav_send_frame(NAV_MSG_SCAN_ACK, payload, sizeof(payload));
}

NUC_NavStatus NUC_Nav_GetStatus(void)
{
  return s_nav_status;
}

uint8_t NUC_Nav_IsOnline(void)
{
  if (s_nav_last_valid_ms == 0U)
  {
    return 0U;
  }
  return (HAL_GetTick() - s_nav_last_valid_ms) < NAV_ONLINE_MS ? 1U : 0U;
}
