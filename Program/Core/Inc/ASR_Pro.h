/**
 * @file ASR_Pro.h
 * @brief STM32 to ASR Pro announcement commands over UART4.
 */
#ifndef ASR_PRO_H
#define ASR_PRO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/** Ask the ASR Pro to announce that bed 1 should collect medicine. */
HAL_StatusTypeDef ASR_Pro_AnnounceBed1(void);

/** Ask the ASR Pro to announce that bed 3 should collect medicine. */
HAL_StatusTypeDef ASR_Pro_AnnounceBed3(void);

#ifdef __cplusplus
}
#endif

#endif /* ASR_PRO_H */
