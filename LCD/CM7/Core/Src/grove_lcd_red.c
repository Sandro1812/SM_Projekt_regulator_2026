/*
 * grove_lcd_red.c
 *
 *  Created on: Jan 9, 2026
 *      Author: malch
 */


#include "grove_lcd_red.h"

// Uchwyt I2C zdefiniowany w main.c (nadal używamy I2C4)
extern I2C_HandleTypeDef hi2c4;

// Adresy I2C (przesunięte dla HAL)
#define LCD_ADDR  0x7C
#define RGB_ADDR  0xC4  // Adres sterownika LED (nawet jeśli tylko czerwony)

// Rejestry sterownika podświetlenia
#define REG_MODE1       0x00
#define REG_MODE2       0x01
#define REG_OUTPUT      0x08
#define REG_RED         0x04 // PWM dla koloru czerwonego

// === Funkcje prywatne ===

static void LCD_SendCmd(uint8_t cmd) {
    uint8_t data[2] = {0x80, cmd};
    HAL_I2C_Master_Transmit(&hi2c4, LCD_ADDR, data, 2, 10);
}

static void LCD_SendData(uint8_t data_byte) {
    uint8_t data[2] = {0x40, data_byte};
    HAL_I2C_Master_Transmit(&hi2c4, LCD_ADDR, data, 2, 10);
}

static void LED_SetReg(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    HAL_I2C_Master_Transmit(&hi2c4, RGB_ADDR, data, 2, 10);
}

// === Funkcje publiczne ===

void GroveLCD_Init(void) {
    HAL_Delay(50);

    // Inicjalizacja sterownika LCD (JHD1802)
    LCD_SendCmd(0x28); // 2 linie, 5x8 punktów
    HAL_Delay(5);
    LCD_SendCmd(0x0C); // Ekran ON, Kursor OFF
    HAL_Delay(1);
    LCD_SendCmd(0x01); // Wyczyść ekran
    HAL_Delay(2);
    LCD_SendCmd(0x06); // Tryb inkrementacji kursora

    // Inicjalizacja sterownika LED
    LED_SetReg(REG_MODE1, 0x00);
    LED_SetReg(REG_OUTPUT, 0xFF); // Enable PWM
    LED_SetReg(REG_MODE2, 0x20);  // Blink mode (wymagane)

    // Włącz podświetlenie na 100%
    GroveLCD_SetBacklight(255);
}

void GroveLCD_Clear(void) {
    LCD_SendCmd(0x01);
    HAL_Delay(2);
}

void GroveLCD_SetCursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0 ? 0x80 : 0xC0) + col;
    LCD_SendCmd(addr);
}

void GroveLCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

void GroveLCD_SetBacklight(uint8_t brightness) {
    // Ustawiamy tylko rejestr czerwony, bo to jedyne diody w tym modelu
    LED_SetReg(REG_RED, brightness);
}
