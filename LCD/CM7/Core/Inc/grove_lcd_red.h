/*
 * grove_lcd_red.h
 *
 *  Created on: Jan 9, 2026
 *      Author: malch
 */

#ifndef INC_GROVE_LCD_RED_H_
#define INC_GROVE_LCD_RED_H_
#include "stm32h7xx_hal.h"

// Funkcje sterujące
void GroveLCD_Init(void);
void GroveLCD_Clear(void);
void GroveLCD_SetCursor(uint8_t col, uint8_t row);
void GroveLCD_Print(char *str);

// Sterowanie podświetleniem (0 = wyłączone, 255 = max jasność)
void GroveLCD_SetBacklight(uint8_t brightness);


#endif /* INC_GROVE_LCD_RED_H_ */
