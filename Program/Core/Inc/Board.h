/**
 * @file Board.h
 * @brief Board-level I/O helpers.
 */
#ifndef BOARD_H
#define BOARD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void BUZZ_On(void);
void BUZZ_Off(void);
void BUZZ_Beep(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
