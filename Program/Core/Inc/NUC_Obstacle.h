/**
 * @file NUC_Obstacle.h
 * @brief NUC obstacle detection UART communication @ UART8 PE1=TX PE0=RX 115200 8N1
 *
 * Frame format (8 bytes):
 * AA BB XH XL YH YL CHK 55
 * - AA BB: frame header
 * - XH XL: forward distance, int16 big-endian, unit mm (positive = forward)
 * - YH YL: lateral offset, int16 big-endian, unit mm (positive = left, negative = right)
 * - CHK: checksum, (XH+XL+YH+YL) & 0xFF
 * - 55: frame tail
 *
 * Link probe:
 *   STM32 -> NUC: A5 5A 01 00
 *   NUC   -> STM32: A5 5A 01 01
 *
 * Usage:
 *   main: MX_UART8_Init(); NUC_Obstacle_Init();
 *         while (!NUC_IsOnline()) {
 *           NUC_Obstacle_SendPing();
 *           HAL_Delay(100);
 *         }
 *   other: int16_t x = NUC_GetObstacleX(); int16_t y = NUC_GetObstacleY();
 *          uint8_t valid = NUC_IsObstacleValid();
 */
#ifndef NUC_OBSTACLE_H
#define NUC_OBSTACLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart8;

/* Debug variables */
extern volatile uint8_t g_nuc_debug_buf[8];
extern volatile uint8_t g_nuc_parse_fail_reason;
extern volatile uint32_t g_nuc_rx_count;
extern volatile uint8_t g_nuc_last_byte;
extern volatile uint8_t g_nuc_rx_history[16];
extern volatile uint8_t g_nuc_history_idx;
extern volatile uint32_t g_nuc_uart_error_count;
extern volatile uint32_t g_nuc_last_uart_error;
extern volatile uint8_t g_nuc_online;

void NUC_Obstacle_Init(void);

/** Send the link probe to NUC (A5 5A 01 00). */
HAL_StatusTypeDef NUC_Obstacle_SendPing(void);

/** Return 1 after NUC has replied to a link probe. */
uint8_t NUC_IsOnline(void);

/** Send probes until NUC replies or timeout_ms expires. */
uint8_t NUC_Obstacle_WaitForOnline(uint32_t timeout_ms);

/** Get obstacle forward distance in mm (0 if no valid data) */
int16_t NUC_GetObstacleX(void);

/** Get obstacle lateral offset in mm (0 if no valid data) */
int16_t NUC_GetObstacleY(void);

/** Check if obstacle data is valid (received within last 200ms) */
uint8_t NUC_IsObstacleValid(void);

/** Internal: UART8 RX interrupt complete, called by HAL_UART_RxCpltCallback */
void NUC_Obstacle_OnUartRxCplt(void);

/** Internal: UART8 error callback, used to recover blocking receive errors */
void NUC_Obstacle_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif /* NUC_OBSTACLE_H */
