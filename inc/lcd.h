#pragma once

#include <stdint.h>

#define LCD_ADDRESS 0x27
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define LCD_ENABLE 0x04
#define LCD_COMMAND 0x00
#define LCD_DATA 0x01

void lcd_init(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_send_string(char *str);

void lcd_init_pitch_yaw(void);
void lcd_update_yaw(char *yaw_angle);
void lcd_update_pitch(char *pitch_angle);
