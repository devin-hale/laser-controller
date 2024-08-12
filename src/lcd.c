#include "lcd.h"
#include "i2c.h"
#include "utils.h"

static void lcd_write_nibble(uint8_t data, uint8_t control) {
  uint8_t highnib = data & 0xF0;
  uint8_t buffer[2];
  buffer[0] = highnib | control | LCD_ENABLE | LCD_BACKLIGHT;
  buffer[1] = highnib | control | LCD_BACKLIGHT;
  i2c_write(LCD_ADDRESS, buffer, 2);
}

static void lcd_send_command(uint8_t command) {
  lcd_write_nibble(command, LCD_COMMAND);
  lcd_write_nibble(command << 4, LCD_COMMAND);
}

static void lcd_send_data(uint8_t data) {
  lcd_write_nibble(data, LCD_DATA);
  lcd_write_nibble(data << 4, LCD_DATA);
}

void lcd_init(void) {
  delay(1000); // Wait for LCD to power up

  // Initialize LCD in 4-bit mode
  lcd_write_nibble(0x30, LCD_COMMAND);
  delay(50);
  lcd_write_nibble(0x30, LCD_COMMAND);
  delay(10);
  lcd_write_nibble(0x30, LCD_COMMAND);
  delay(10);
  lcd_write_nibble(0x20, LCD_COMMAND); // Set to 4-bit mode

  //// Function Set
  lcd_send_command(0x28); // 4-bit mode, 2 lines, 5x8 font

  //// Display Control
  lcd_send_command(0x08); // Display off, cursor off, blink off

  //// Clear Display
  lcd_send_command(0x01); // Clear display

  //// Entry Mode Set
  lcd_send_command(0x06); // Increment cursor, no shift

  //// Display On
  lcd_send_command(0x0C); // Display on, cursor off, blink off

  delay(50); // Wait for the LCD to process commands
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
  uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row > 1) {
    row = 1; // We only support 2 rows: 0 and 1
  }
  lcd_send_command(0x80 | (col + row_offsets[row]));
}

void lcd_send_string(char *str) {
  while (*str) {
    lcd_send_data(*str++);
  }
}

#define YAW_ROW 0
#define PITCH_ROW 1
#define YAW_START_POS 5
#define PITCH_START_POS 7
static char *blank_value = "   ";

void lcd_init_pitch_yaw(void) {
  lcd_set_cursor(0, YAW_ROW);
  lcd_send_string("YAW:");
  lcd_set_cursor(0, PITCH_ROW);
  lcd_send_string("PITCH:");
}

void lcd_update_yaw(char *yaw_angle) {
  lcd_set_cursor(YAW_START_POS, YAW_ROW);
  lcd_send_string(blank_value);
  lcd_set_cursor(YAW_START_POS, YAW_ROW);
  lcd_send_string(yaw_angle);
}

void lcd_update_pitch(char *pitch_angle) {
  lcd_set_cursor(PITCH_START_POS, PITCH_ROW);
  lcd_send_string(blank_value);
  lcd_set_cursor(PITCH_START_POS, PITCH_ROW);
  lcd_send_string(pitch_angle);
}
