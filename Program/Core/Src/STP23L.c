#include "STP23L.h"
#include <string.h>

#define STP23L_FRAME_SIZE 195U
#define STP23L_HEADER_SIZE 4U
#define STP23L_POINT_COUNT 12U
#define STP23L_POINT_SIZE 15U
#define STP23L_DATA_OFFSET 10U
#define STP23L_PAYLOAD_SIZE 184U
#define STP23L_MIN_DISTANCE_MM 100U
#define STP23L_MAX_DISTANCE_MM 13400U
#define STP23L_TIMEOUT_MS 500U

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t rx_byte;
  uint8_t frame[STP23L_FRAME_SIZE];
  uint16_t frame_index;
  uint8_t header_count;
  volatile uint16_t distance_mm;
  volatile uint32_t last_frame_ms;
  volatile uint32_t frame_sequence;
  volatile uint8_t has_frame;
} STP23L_Channel;

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef hlpuart1;

static STP23L_Channel s_channel_a;
static STP23L_Channel s_channel_b;
static STP23L_Channel s_channel_c;

static void STP23L_ResetParser(STP23L_Channel *channel)
{
  channel->frame_index = 0U;
  channel->header_count = 0U;
}

static uint8_t STP23L_FrameIsValid(const uint8_t *frame)
{
  uint16_t payload_size;
  uint8_t checksum = 0U;
  uint16_t index;

  if ((frame[0] != 0xAAU) || (frame[1] != 0xAAU) ||
      (frame[2] != 0xAAU) || (frame[3] != 0xAAU))
  {
    return 0U;
  }

  payload_size = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8U);
  if ((frame[4] != 0x00U) || (frame[5] != 0x02U) ||
      (frame[6] != 0x00U) || (frame[7] != 0x00U) ||
      (payload_size != STP23L_PAYLOAD_SIZE))
  {
    return 0U;
  }

  for (index = 4U; index < (STP23L_FRAME_SIZE - 1U); ++index)
  {
    checksum = (uint8_t)(checksum + frame[index]);
  }

  return (checksum == frame[STP23L_FRAME_SIZE - 1U]) ? 1U : 0U;
}

static void STP23L_ApplyFrame(STP23L_Channel *channel)
{
  uint32_t distance_sum = 0U;
  uint16_t point_index;
  uint8_t valid_count = 0U;

  for (point_index = 0U; point_index < STP23L_POINT_COUNT; ++point_index)
  {
    uint16_t offset = STP23L_DATA_OFFSET + (point_index * STP23L_POINT_SIZE);
    uint16_t distance = (uint16_t)channel->frame[offset] |
                        ((uint16_t)channel->frame[offset + 1U] << 8U);

    if ((distance >= STP23L_MIN_DISTANCE_MM) &&
        (distance <= STP23L_MAX_DISTANCE_MM))
    {
      distance_sum += distance;
      ++valid_count;
    }
  }

  channel->distance_mm = (valid_count > 0U)
                             ? (uint16_t)(distance_sum / valid_count)
                             : 0U;
  channel->last_frame_ms = HAL_GetTick();
  channel->frame_sequence++;
  channel->has_frame = 1U;
}

static void STP23L_KeepTrailingHeader(STP23L_Channel *channel)
{
  uint8_t trailing_count = 0U;
  uint16_t index = STP23L_FRAME_SIZE;

  while ((index > 0U) && (trailing_count < STP23L_HEADER_SIZE) &&
         (channel->frame[index - 1U] == 0xAAU))
  {
    --index;
    ++trailing_count;
  }

  channel->frame_index = 0U;
  channel->header_count = trailing_count;
  if (trailing_count == STP23L_HEADER_SIZE)
  {
    memset(channel->frame, 0xAA, STP23L_HEADER_SIZE);
    channel->frame_index = STP23L_HEADER_SIZE;
    channel->header_count = 0U;
  }
}

static void STP23L_FeedByte(STP23L_Channel *channel, uint8_t byte)
{
  if (channel->frame_index == 0U)
  {
    if (byte == 0xAAU)
    {
      ++channel->header_count;
      if (channel->header_count == STP23L_HEADER_SIZE)
      {
        memset(channel->frame, 0xAA, STP23L_HEADER_SIZE);
        channel->frame_index = STP23L_HEADER_SIZE;
        channel->header_count = 0U;
      }
    }
    else
    {
      channel->header_count = 0U;
    }
    return;
  }

  channel->frame[channel->frame_index] = byte;
  ++channel->frame_index;

  if (channel->frame_index == STP23L_FRAME_SIZE)
  {
    if (STP23L_FrameIsValid(channel->frame) != 0U)
    {
      STP23L_ApplyFrame(channel);
      STP23L_ResetParser(channel);
    }
    else
    {
      STP23L_KeepTrailingHeader(channel);
    }
  }
}

static STP23L_Channel *STP23L_FindChannel(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    return &s_channel_a;
  }
  if (huart->Instance == UART5)
  {
    return &s_channel_b;
  }
  if (huart->Instance == LPUART1)
  {
    return &s_channel_c;
  }
  return NULL;
}

static uint8_t STP23L_IsFresh(const STP23L_Channel *channel)
{
  if (channel->has_frame == 0U)
  {
    return 0U;
  }
  return ((uint32_t)(HAL_GetTick() - channel->last_frame_ms) <=
          STP23L_TIMEOUT_MS)
             ? 1U
             : 0U;
}

static uint16_t STP23L_GetDistance(const STP23L_Channel *channel)
{
  return (STP23L_IsFresh(channel) != 0U) ? channel->distance_mm : 0U;
}

static uint8_t STP23L_GetSample(const STP23L_Channel *channel,
                                uint16_t *distance_mm,
                                uint32_t *frame_sequence)
{
  uint16_t distance;
  uint32_t sequence;
  uint32_t last_frame_ms;
  uint8_t has_frame;
  uint32_t primask;

  if ((distance_mm == NULL) || (frame_sequence == NULL))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  distance = channel->distance_mm;
  sequence = channel->frame_sequence;
  last_frame_ms = channel->last_frame_ms;
  has_frame = channel->has_frame;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((has_frame == 0U) || (distance == 0U) ||
      ((uint32_t)(HAL_GetTick() - last_frame_ms) > STP23L_TIMEOUT_MS))
  {
    return 0U;
  }

  *distance_mm = distance;
  *frame_sequence = sequence;
  return 1U;
}

void STP23L_Init(void)
{
  memset(&s_channel_a, 0, sizeof(s_channel_a));
  memset(&s_channel_b, 0, sizeof(s_channel_b));
  memset(&s_channel_c, 0, sizeof(s_channel_c));

  s_channel_a.uart = &huart6;
  s_channel_b.uart = &huart5;
  s_channel_c.uart = &hlpuart1;

  (void)HAL_UART_Receive_IT(s_channel_a.uart, &s_channel_a.rx_byte, 1U);
  (void)HAL_UART_Receive_IT(s_channel_b.uart, &s_channel_b.rx_byte, 1U);
  (void)HAL_UART_Receive_IT(s_channel_c.uart, &s_channel_c.rx_byte, 1U);
}

void STP23L_OnUartRxCplt(UART_HandleTypeDef *huart)
{
  STP23L_Channel *channel = STP23L_FindChannel(huart);

  if (channel == NULL)
  {
    return;
  }

  STP23L_FeedByte(channel, channel->rx_byte);
  (void)HAL_UART_Receive_IT(channel->uart, &channel->rx_byte, 1U);
}

void STP23L_OnUartError(UART_HandleTypeDef *huart)
{
  STP23L_Channel *channel = STP23L_FindChannel(huart);

  if (channel == NULL)
  {
    return;
  }

  STP23L_ResetParser(channel);
  (void)HAL_UART_Receive_IT(channel->uart, &channel->rx_byte, 1U);
}

uint16_t STP32_getA(void)
{
  return STP23L_GetDistance(&s_channel_a);
}

uint16_t STP32_getB(void)
{
  return STP23L_GetDistance(&s_channel_b);
}

uint16_t STP32_getC(void)
{
  return STP23L_GetDistance(&s_channel_c);
}

uint8_t STP23L_IsOnlineA(void)
{
  return STP23L_IsFresh(&s_channel_a);
}

uint8_t STP23L_IsOnlineB(void)
{
  return STP23L_IsFresh(&s_channel_b);
}

uint8_t STP23L_IsOnlineC(void)
{
  return STP23L_IsFresh(&s_channel_c);
}

uint8_t STP23L_GetSampleA(uint16_t *distance_mm, uint32_t *frame_sequence)
{
  return STP23L_GetSample(&s_channel_a, distance_mm, frame_sequence);
}

uint8_t STP23L_GetSampleB(uint16_t *distance_mm, uint32_t *frame_sequence)
{
  return STP23L_GetSample(&s_channel_b, distance_mm, frame_sequence);
}

uint8_t STP23L_GetSampleC(uint16_t *distance_mm, uint32_t *frame_sequence)
{
  return STP23L_GetSample(&s_channel_c, distance_mm, frame_sequence);
}
