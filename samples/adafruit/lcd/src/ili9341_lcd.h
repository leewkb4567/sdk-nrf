/*
 * Copyright (c) 2015 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef ILI9341_LCD_H__
#define ILI9341_LCD_H__

// ILI9341 LCD size is QVGA.
#define ILI9341_LCD_WIDTH     320
#define ILI9341_LCD_HEIGHT    240

#define LCD_COLOR_BLACK		0
#define LCD_COLOR_RED		1
#define LCD_COLOR_GREEN		2
#define LCD_COLOR_YELLOW	3
#define LCD_COLOR_BLUE		4
#define LCD_COLOR_MAGENTA 	5
#define LCD_COLOR_CYAN		6
#define LCD_COLOR_WHITE		7

/**@brief Function to initiate LCD.
 */
void ili9341_lcd_init(void);

/**@brief Function to turn on LCD.
 */
void ili9341_lcd_on(void);

/**@brief Function to turn off LCD.
 */
void ili9341_lcd_off(void);

/**@brief Function to fill the entire LCD.
 */
void ili9341_lcd_fill(int color8);

/**@brief Function to put a big dot on LCD.
 */
void ili9341_lcd_put_dot(int x, int y, int color8);

typedef struct {
	int w;
	int h;
} lcd_dot_size_t;

/**@brief Function to get LCD dot size.
 */
lcd_dot_size_t ili9341_lcd_get_dot_size(void);

#endif
