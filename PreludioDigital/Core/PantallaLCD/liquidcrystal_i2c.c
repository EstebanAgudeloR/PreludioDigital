/*
 * liquidcrystal_i2c.c
 *
 *  Created on: Jul 10, 2025
 *      Author: Esteban Agudelo Rincón
 */

#include "liquidcrystal_i2c.h"

extern I2C_HandleTypeDef hi2c1; // Asegúrate que es tu handler de I2C

uint8_t _addr;
uint8_t _cols;
uint8_t _rows;
uint8_t _backlightval;

void I2C_send(uint8_t data, uint8_t flags);
void LCD_write_nibble(uint8_t nibble, uint8_t rs_flag);
void LCD_send_cmd(uint8_t cmd);
void LCD_send_data(uint8_t data);

void LiquidCrystal_I2C_Init(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows) {
    _addr = lcd_addr << 1;
    _cols = lcd_cols;
    _rows = lcd_rows;
    _backlightval = 0x08; // Backlight ON

    HAL_Delay(50);
    LCD_send_cmd(0x33);
    HAL_Delay(5);
    LCD_send_cmd(0x32);
    HAL_Delay(5);
    LCD_send_cmd(0x28); // 4-bit mode, 2 lines, 5x8 font
    HAL_Delay(5);
    LCD_send_cmd(0x0C); // Display ON, Cursor OFF, Blink OFF
    HAL_Delay(5);
    LCD_send_cmd(0x01); // Clear display
    HAL_Delay(5);
    LCD_send_cmd(0x06); // Entry mode: increment cursor, no shift
}

void I2C_send(uint8_t data, uint8_t flags) {
    uint8_t packet[1];
    packet[0] = data | flags | _backlightval;
    HAL_I2C_Master_Transmit(&hi2c1, _addr, packet, 1, 100);
}

void LCD_write_nibble(uint8_t nibble, uint8_t rs_flag) {
    uint8_t data = nibble << 4;
    I2C_send(data, rs_flag);
    I2C_send(data, rs_flag | 0x04); // Enable pulse
    I2C_send(data, rs_flag);        // End of pulse
}

void LCD_send_cmd(uint8_t cmd) {
    uint8_t upper_nibble = cmd >> 4;
    uint8_t lower_nibble = cmd & 0x0F;
    LCD_write_nibble(upper_nibble, 0x00);
    LCD_write_nibble(lower_nibble, 0x00);
}

void LCD_send_data(uint8_t data) {
    uint8_t upper_nibble = data >> 4;
    uint8_t lower_nibble = data & 0x0F;
    LCD_write_nibble(upper_nibble, 0x01);
    LCD_write_nibble(lower_nibble, 0x01);
}

void LiquidCrystal_I2C_Clear() {
    LCD_send_cmd(0x01);
    HAL_Delay(5);
}

void LiquidCrystal_I2C_Home() {
    LCD_send_cmd(0x02);
    HAL_Delay(5);
}

void LiquidCrystal_I2C_SetCursor(uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= _rows) {
        row = _rows - 1;
    }
    LCD_send_cmd(0x80 | (col + row_offsets[row]));
}

void LiquidCrystal_I2C_Print(char* str) {
    while (*str) {
        LCD_send_data((uint8_t)(*str));
        str++;
    }
}

void LiquidCrystal_I2C_Backlight() {
    _backlightval = 0x08;
    I2C_send(0,0); // Send a dummy byte to update backlight
}

void LiquidCrystal_I2C_NoBacklight() {
    _backlightval = 0x00;
    I2C_send(0,0); // Send a dummy byte to update backlight
}
