/**
 * @file CN_TTS.h
 * @brief CN-TTS speech synthesis module on UART7, PE8/PE7, 9600 8N1.
 */
#ifndef CN_TTS_H
#define CN_TTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define CN_TTS_MAX_TEXT_BYTES 150U

/** Start receiving the module's A(started)/O(finished) status bytes. */
void CN_TTS_Init(void);

/** Send GBK text for synthesis. ASCII may be mixed into the byte stream. */
HAL_StatusTypeDef CN_TTS_SpeakGBK(const uint8_t *text, uint16_t length);

/** Configure the documented volume range, 1 (lowest) through 4 (highest). */
HAL_StatusTypeDef CN_TTS_SetVolume(uint8_t volume);

/** Configure the documented speech-rate range, 1 through 3. */
HAL_StatusTypeDef CN_TTS_SetSpeed(uint8_t speed);

/** Play one of the built-in effects, numbered 0 through 7. */
HAL_StatusTypeDef CN_TTS_PlaySound(uint8_t sound);

/** Announce that the patient at bed 1 should collect medicine. */
HAL_StatusTypeDef CN_TTS_AnnounceBed1(void);

/** Announce that the patient at bed 3 should collect medicine. */
HAL_StatusTypeDef CN_TTS_AnnounceBed3(void);

/** Nonzero after transmission starts until the module returns 'O'. */
uint8_t CN_TTS_IsBusy(void);

/** Internal UART7 receive-complete hook. */
void CN_TTS_OnUartRxCplt(void);

/** Internal UART7 error-recovery hook. */
void CN_TTS_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif /* CN_TTS_H */
