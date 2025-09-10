/*
 * liquidcrystal_i2c.h
 *
 *  Created on: Jul 10, 2025
 *      Author: Esteban Agudelo Rincón
 */

#ifndef DRIVER_LSD_LIQUIDCRYSTAL_I2C_H_
#define DRIVER_LSD_LIQUIDCRYSTAL_I2C_H_

#include "stm32f4xx_hal.h"

void LiquidCrystal_I2C_Init(uint8_t lcd_addr, uint8_t lcd_cols, uint8_t lcd_rows);
void LiquidCrystal_I2C_Clear();
void LiquidCrystal_I2C_Home();
void LiquidCrystal_I2C_SetCursor(uint8_t col, uint8_t row);
void LiquidCrystal_I2C_Print(char* str);
void LiquidCrystal_I2C_Backlight();
void LiquidCrystal_I2C_NoBacklight();

#endif /* DRIVER_LSD_LIQUIDCRYSTAL_I2C_H_ */
